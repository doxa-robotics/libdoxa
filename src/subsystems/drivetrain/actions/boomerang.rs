use core::f64::consts::PI;

use pid::Pid;

use crate::{
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

#[derive(Debug, Clone, Copy)]
pub struct BoomerangAction {
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    target_point: Pose,
    left_zero: Option<f64>,
    right_zero: Option<f64>,
    settling_distance: f64,
}

impl BoomerangAction {
    pub fn new(target_point: Pose, config: super::config::ActionConfig) -> Self {
        Self {
            rotational_pid: config.turn_pid(0.0),
            linear_pid: config.linear_pid(0.0),
            linear_tolerances: config.linear_tolerances(),
            target_point,
            left_zero: None,
            right_zero: None,
            settling_distance: 50.0,
        }
    }

    pub fn with_settling_distance(&mut self, settling_distance: f64) -> Self {
        self.settling_distance = settling_distance;
        *self
    }
}

impl super::Action for BoomerangAction {
    fn update(&mut self, context: super::ActionContext) -> Option<VoltagePair> {
        let distance = self.target_point.distance(context.pose);
        if distance > self.settling_distance
            || (self.left_zero.is_none() || self.right_zero.is_none())
        {
            // If we are more than 50 mm away from the target, we need to set the setpoint
            // to the distance to the target.
            self.linear_pid.setpoint(distance);
            self.left_zero = Some(context.left_offset);
            self.right_zero = Some(context.right_offset);
        } else {
            // If we are within 50 mm of the target, we don't set the setpoint
            // since there might be oscillation.
        }
        let mut linear_voltage = self
            .linear_pid
            .next_control_output(
                (context.left_offset - self.left_zero.unwrap_or(0.0) + context.right_offset
                    - self.right_zero.unwrap_or(0.0))
                    / 2.0,
            )
            .output;
        if self.linear_tolerances.check(
            distance,
            (context.left_velocity + context.right_velocity) / 2.0,
        ) {
            return None;
        }
        let mut angle_to_target = context.pose.angle_to(self.target_point);
        // Adjust angle_to_target to be closest to the context.pose.heading()
        while angle_to_target - context.pose.heading() > PI {
            angle_to_target -= 2.0 * PI;
        }
        while angle_to_target - context.pose.heading() < -PI {
            angle_to_target += 2.0 * PI;
        }
        if angle_to_target - context.pose.heading() > PI / 2.0 {
            angle_to_target -= PI;
            linear_voltage = -linear_voltage;
        } else if angle_to_target - context.pose.heading() < -PI / 2.0 {
            angle_to_target += PI;
            linear_voltage = -linear_voltage;
        }
        while angle_to_target - context.pose.heading() > PI {
            angle_to_target -= 2.0 * PI;
        }
        while angle_to_target - context.pose.heading() < -PI {
            angle_to_target += 2.0 * PI;
        }
        // If we're within 50 mm of the target, we don't want to update our setpoint.
        if distance > self.settling_distance {
            self.rotational_pid.setpoint(angle_to_target);
        }
        let rotational_voltage = self
            .rotational_pid
            .next_control_output(context.pose.heading())
            .output;
        // If we are not settled, we need to return the voltage
        Some(VoltagePair {
            left: linear_voltage - rotational_voltage,
            right: linear_voltage + rotational_voltage,
        })
    }
}
