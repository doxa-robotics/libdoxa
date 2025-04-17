use pid::Pid;

use crate::utils::settling;

/// An action that drives the robot forward a certain distance.
///
/// This action uses a PID controller to drive the robot forward a certain distance.
/// Set the PID constants and set point to determine the target distance.
#[derive(Debug)]
pub struct ForwardAction {
    controller: Pid<f64>,
    tolerances: settling::Tolerances,

    initial_left_offset: Option<f64>,
    initial_right_offset: Option<f64>,
}

impl ForwardAction {
    pub fn new(controller: Pid<f64>, tolerances: settling::Tolerances) -> Self {
        Self {
            controller,
            tolerances,
            initial_left_offset: None,
            initial_right_offset: None,
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
    ) -> Option<crate::subsystems::drivetrain::VoltagePair> {
        if self.initial_left_offset.is_none() {
            self.initial_left_offset = Some(context.left_offset);
        }
        if self.initial_right_offset.is_none() {
            self.initial_right_offset = Some(context.right_offset);
        }
        let average_distance = (context.left_offset - self.initial_left_offset.unwrap_or(0.0)
            + context.right_offset
            - self.initial_right_offset.unwrap_or(0.0))
            / 2.0;
        let error = self.controller.setpoint - average_distance;
        if self.tolerances.check(
            error,
            (context.left_velocity + context.right_velocity) / 2.0,
        ) {
            return None;
        }

        let output = self.controller.next_control_output(average_distance).output;

        log::debug!(
            "Distance: {:.2?} Control output: {:.2?} Error: {:.2?} Setpoint: {:.2?} heading: {:.2?}",
            average_distance,
            output,
            error,
            self.controller.setpoint,
            context.pose.heading()
        );

        Some(output.into())
    }
}
