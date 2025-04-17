use core::f64::consts::PI;

use pid::Pid;

use crate::{
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

#[derive(Debug)]
pub struct BoomerangAction {
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    target_point: Pose,
}

impl BoomerangAction {
    pub fn new(
        rotational_pid: Pid<f64>,
        mut linear_pid: Pid<f64>,
        linear_tolerances: Tolerances,
        target_point: Pose,
    ) -> Self {
        linear_pid.setpoint(0.0);
        Self {
            rotational_pid,
            linear_pid,
            linear_tolerances,
            target_point,
        }
    }
}

impl super::Action for BoomerangAction {
    fn update(&mut self, context: super::ActionContext) -> Option<VoltagePair> {
        let distance = self.target_point.distance(context.pose);
        self.linear_pid.setpoint(distance);
        let mut linear_voltage = self.linear_pid.next_control_output(distance).output;
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
        self.rotational_pid.setpoint(angle_to_target);
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
