use crate::utils::settling;
use pid::Pid;

/// An action that rotates the drivetrain to a specific absolute heading.
///
/// This action uses a PID controller to rotate the drivetrain to a target heading.
/// Set the PID constants and set point to determine the target heading.
#[derive(Debug)]
pub struct RotationAction {
    controller: Pid<f64>,
    tolerances: settling::Tolerances,
    initialized: bool, // Flag to track if the setpoint has been normalized
}

impl RotationAction {
    pub fn new(controller: Pid<f64>, tolerances: settling::Tolerances) -> Self {
        Self {
            controller,
            tolerances,
            initialized: false, // Initialize the flag to false
        }
    }

    pub fn controller(&mut self) -> &mut Pid<f64> {
        &mut self.controller
    }

    pub fn tolerances(&self) -> settling::Tolerances {
        self.tolerances
    }
}

impl super::Action for RotationAction {
    fn update(
        &mut self,
        context: super::ActionContext,
    ) -> Option<crate::subsystems::drivetrain::VoltagePair> {
        if !self.initialized {
            // Normalize the setpoint to the closest direct angle to the current position
            let mut error = self.controller.setpoint - context.heading;
            while error > 180.0 {
                self.controller.setpoint -= 360.0;
                error -= 360.0;
            }
            while error < -180.0 {
                self.controller.setpoint += 360.0;
                error += 360.0;
            }
            self.initialized = true; // Mark as initialized
        }

        let error = self.controller.setpoint - context.heading;

        if self
            .tolerances
            .check(error, context.left_velocity - context.right_velocity)
        {
            return None;
        }

        let output = self.controller.next_control_output(context.heading).output;

        log::debug!(
            "Control output: {:.2?} Error: {:.2?} Setpoint: {:.2?} heading: {:.2?}",
            output,
            error,
            self.controller.setpoint,
            context.heading
        );

        // Apply the output as a voltage pair for rotation
        Some(crate::subsystems::drivetrain::VoltagePair {
            left: output,
            right: -output,
        })
    }
}
