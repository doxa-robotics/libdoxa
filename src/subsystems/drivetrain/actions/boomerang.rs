use nalgebra::{Point2, Vector2};
use pid::Pid;
use vexide::math::Angle;

use crate::{subsystems::drivetrain::DrivetrainPair, utils::settling::Tolerances};

// Inspired by https://github.com/vexide/evian/blob/2c07838519f335f2308d7d1b869cb62363f635fb/packages/evian-motion/src/seeking/boomerang.rs

/// An action that moves the drivetrain to a target point using a boomerang
/// approach.
#[derive(Debug, Clone, Copy)]
pub struct BoomerangAction {
    target_point: Point2<f64>,
    target_heading: Angle,
    /// The lead percentage for the carrot (what we're currently aiming for)
    /// (0.0, 1.0]
    lead: f64,
    /// When the distance is "close enough" to the target point, we should stop
    /// seeking the point in the angular dimension.
    /// This prevents oscillation when we are very close to the target point.
    close: f64,

    /// Whether this action is reversed
    reverse: bool,

    tolerances: Tolerances,

    linear_pid: Pid<f64>,
    angular_pid: Pid<f64>,
}

impl BoomerangAction {
    pub fn new(
        target_point: Point2<f64>,
        target_heading: Angle,
        config: super::config::ActionConfig,
    ) -> Self {
        Self {
            target_point,
            target_heading,
            lead: config.boomerang_lead,
            close: config.boomerang_close,
            tolerances: config.linear_tolerances(),
            linear_pid: config.linear_pid(0.0),
            angular_pid: config.turn_pid(0.0),
            reverse: false,
        }
    }

    /// Sets this action to be reversed.
    ///
    /// When reversed, the robot will drive backwards to the target point,
    /// attempting to face the *back* of the robot towards the target heading.
    pub fn reversed(mut self) -> Self {
        self.reverse = true;
        self
    }
}

impl super::Action for BoomerangAction {
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        // Carrot -- what we're currently aiming for
        // We want to aim for a point ahead of the target point in the direction
        // of the target heading, scaled by the distance to the target point.
        let carrot = {
            let distance = nalgebra::distance(&self.target_point, &context.data.offset);
            let carrot_offset = Vector2::new(
                distance * self.target_heading.cos(),
                distance * self.target_heading.sin(),
            ) * self.lead;
            self.target_point - carrot_offset
        };

        let local_target = carrot - context.data.offset;
        let angle_to_target = Angle::from_radians(local_target.y.atan2(local_target.x));

        // Compute the angular angle
        let error_angular = (angle_to_target
            - if self.reverse {
                // If we're reversed, we want to face backwards
                context.data.heading + Angle::HALF_TURN
            } else {
                context.data.heading
            })
        .wrapped_half();
        let (error_distance, close) = {
            let norm = local_target.norm();
            let close = norm < self.close;
            (norm, close)
        };

        // Check tolerances
        if self
            .tolerances
            .check(error_distance, context.data.linear_velocity())
        {
            return None;
        }

        let output_angular = if close {
            // If we're close, angle changes erratically, so just stop turning
            0.0
        } else {
            self.angular_pid
                .next_control_output(error_angular.as_radians())
                .output
        };
        let output_linear = self.linear_pid.next_control_output(-error_distance).output
            // If the angular error is more than 90 degrees (|cos(angle) < 0|),
            // stop moving forward
            * error_angular.cos().max(0.0)
            // If reversed, invert the linear output to drive backwards
            * if self.reverse { -1.0 } else { 1.0 };

        Some(DrivetrainPair {
            left: output_linear - output_angular,
            right: output_linear + output_angular,
            units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::Voltage,
        })
    }
}
