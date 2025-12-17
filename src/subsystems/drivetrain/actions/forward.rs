use nalgebra::Point2;
use pid::Pid;

use crate::{subsystems::drivetrain::DrivetrainPair, utils::settling};

use super::config::ActionConfig;

/// An action that drives the robot forward a certain distance.
///
/// This action uses a PID controller to drive the robot forward a certain
/// distance.
#[derive(Debug)]
pub struct ForwardAction {
    controller: Pid<f64>,
    tolerances: settling::Tolerances,

    initial_point: Option<Point2<f64>>,
}

impl ForwardAction {
    pub fn new(distance: f64, config: ActionConfig) -> Self {
        let controller = config.linear_pid(distance);
        Self {
            controller,
            tolerances: config.linear_tolerances(),
            initial_point: None,
        }
    }

    pub fn controller(&mut self) -> &mut Pid<f64> {
        &mut self.controller
    }

    pub fn tolerances(&self) -> settling::Tolerances {
        self.tolerances
    }
}

impl super::Action for ForwardAction {
    fn update(
        &mut self,
        context: super::ActionContext,
    ) -> Option<crate::subsystems::drivetrain::DrivetrainPair> {
        if self.initial_point.is_none() {
            self.initial_point = Some(context.data.offset);
        }

        let travelled = context.data.offset - self.initial_point.unwrap();
        let mut distance = travelled.norm();
        if self.controller.setpoint < 0.0 {
            // If we are going backwards, invert the distance
            distance *= -1.0;
        }
        let error = self.controller.setpoint - distance;
        if self.tolerances.check(error, context.data.linear_velocity()) {
            return None;
        }

        let output = self.controller.next_control_output(distance).output;

        Some(DrivetrainPair::from(output))
    }
}
