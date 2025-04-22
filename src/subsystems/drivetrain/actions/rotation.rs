use core::f64::consts::PI;

use crate::utils::settling;
use pid::Pid;

use super::config::ActionConfig;

/// An action that rotates the drivetrain to a specific absolute heading.
///
/// This action uses a PID controller to rotate the drivetrain to a target heading.
#[derive(Debug)]
pub struct RotationAction {
    controller: Pid<f64>,
    tolerances: settling::Tolerances,
    initialized: bool, // Flag to track if the setpoint has been normalized
}

impl RotationAction {
    pub fn new(target_radians: f64, config: ActionConfig) -> Self {
        Self {
            controller: config.turn_pid(target_radians),
            tolerances: config.turn_tolerances(),
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
            let mut error = self.controller.setpoint - context.pose.heading();
            while error > PI {
                self.controller.setpoint -= 2.0 * PI;
                error -= 2.0 * PI;
            }
            while error < -PI {
                self.controller.setpoint += 2.0 * PI;
                error += 2.0 * PI;
            }
            self.initialized = true; // Mark as initialized
        }

        let error = self.controller.setpoint - context.pose.heading();

        if self
            .tolerances
            .check(error, context.left_velocity - context.right_velocity)
        {
            return None;
        }

        let output = self
            .controller
            .next_control_output(context.pose.heading())
            .output;

        log::debug!(
            "Control output: {:.2?} Error: {:.2?} Setpoint: {:.2?} heading: {:.2?}",
            output,
            error,
            self.controller.setpoint,
            context.pose.heading()
        );

        // Apply the output as a voltage pair for rotation
        Some(crate::subsystems::drivetrain::VoltagePair {
            left: output,
            right: -output,
        })
    }
}
