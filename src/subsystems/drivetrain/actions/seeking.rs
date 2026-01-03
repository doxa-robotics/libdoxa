use nalgebra::Point2;
use pid::Pid;
use vexide::math::Angle;

use crate::{subsystems::drivetrain::DrivetrainPair, utils::settling::Tolerances};

#[derive(Debug, Clone, Copy)]
pub struct SeekingAction {
    target_point: Point2<f64>,
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

impl SeekingAction {
    pub fn new(target_point: Point2<f64>, config: super::config::ActionConfig) -> Self {
        Self {
            target_point,
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

impl super::Action for SeekingAction {
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        let local_target = self.target_point - context.data.offset;
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
