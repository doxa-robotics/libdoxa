use core::f64::consts::PI;

use nalgebra::{Point2, Vector2};
use pid::Pid;
use vexide::math::Angle;

use crate::{subsystems::drivetrain::DrivetrainPair, utils::settling::Tolerances};

/// Returns a scalar to apply to the linear output based on the turn error.
pub(super) fn turning_linear_scalar_curve(turn_error: Angle) -> f64 {
    // // min(1.0/(abs(x) + 0.8), 1.0)
    // // Domain: (-PI, PI)
    // // Range: (0.0, 1.0)
    // 1.0_f64.min(1.0 / (turn_error.abs() + 0.8))
    turn_error.as_radians().cos().abs()
}

#[derive(Debug, Clone, Copy)]
pub struct BoomerangAction {
    rotational_pid: Pid<f64>,
    rotational_setpoint_initialized: bool,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    target_point: Point2<f64>,
    settling_distance: f64,
}

impl BoomerangAction {
    pub fn new(target_point: Point2<f64>, config: super::config::ActionConfig) -> Self {
        Self {
            rotational_pid: config.turn_pid(0.0),
            linear_pid: config.linear_pid(0.0),
            linear_tolerances: config.linear_tolerances(),
            target_point,
            rotational_setpoint_initialized: false,
            settling_distance: config.boomerang_lock_distance.unwrap_or(200.0),
        }
    }

    pub fn with_settling_distance(&mut self, settling_distance: f64) -> Self {
        self.settling_distance = settling_distance;
        *self
    }
}

impl super::Action for BoomerangAction {
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        let distance = nalgebra::distance(&self.target_point, &context.data.offset);

        let mut angle_to_target = Angle::from_radians(
            (self.target_point.y - context.data.offset.y)
                .atan2(self.target_point.x - context.data.offset.x),
        );
        // Adjust angle_to_target to be closest to the context.pose.heading()
        while angle_to_target - context.data.heading > Angle::HALF_TURN {
            angle_to_target -= Angle::FULL_TURN;
        }
        while angle_to_target - context.data.heading < -Angle::HALF_TURN {
            angle_to_target += Angle::FULL_TURN;
        }
        let mut inverted = false;
        if angle_to_target - context.data.heading > Angle::from_radians(PI / 2.0) {
            angle_to_target -= Angle::from_radians(PI);
            inverted = true;
        } else if angle_to_target - context.data.heading < -Angle::from_radians(PI / 2.0) {
            angle_to_target += Angle::from_radians(PI);
            inverted = true;
        }
        while angle_to_target - context.data.heading > Angle::from_radians(PI) {
            angle_to_target -= Angle::from_radians(2.0 * PI);
        }
        while angle_to_target - context.data.heading < -Angle::from_radians(PI) {
            angle_to_target += Angle::from_radians(2.0 * PI);
        }
        // If we're within 50 mm of the target, we don't want to update our setpoint.
        if distance > self.settling_distance || !self.rotational_setpoint_initialized {
            self.rotational_pid.setpoint(angle_to_target.as_radians());
            self.rotational_setpoint_initialized = true;
        }
        let rotational_voltage = self
            .rotational_pid
            .next_control_output(context.data.heading.as_radians())
            .output;

        if distance > self.settling_distance {
            // If we are more than 50 mm away from the target, we need to set the setpoint
            // to the distance to the target.
            self.linear_pid
                .setpoint(if inverted { -distance } else { distance });

            // TODO: compute turn first so we can invert setpoint
        } else {
            // If we are within 50 mm of the target, we don't set the setpoint
            // since there might be oscillation.
            // Let's just use the dot product to determine how much to move
            // forward/backward.
            let heading_vector = Vector2::new(
                context.data.heading.as_radians().cos(),
                context.data.heading.as_radians().sin(),
            );
            let to_target_vector = Vector2::new(
                self.target_point.x - context.data.offset.x,
                self.target_point.y - context.data.offset.y,
            );
            let dot_product = heading_vector.dot(&to_target_vector);
            self.linear_pid.setpoint(dot_product);
        }
        let mut linear_voltage = self
            .linear_pid
            .next_control_output(nalgebra::distance(&self.target_point, &context.data.offset))
            .output;
        if self.linear_tolerances.check(
            nalgebra::distance(&self.target_point, &context.data.offset),
            context.data.linear_velocity(),
        ) {
            return None;
        }
        // If we are not settled, we need to return the voltage
        // First, scale the linear voltage.
        let linear_scalar = turning_linear_scalar_curve(context.data.heading - angle_to_target);
        linear_voltage *= linear_scalar;
        Some(DrivetrainPair {
            left: linear_voltage - rotational_voltage,
            right: linear_voltage + rotational_voltage,
            units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::RPM,
        })
    }
}
