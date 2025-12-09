use crate::utils::settling;
use pid::Pid;
use vexide::math::Angle;

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
    ) -> Option<crate::subsystems::drivetrain::DrivetrainPair> {
        // Calculate the shortest angular error
        let error = (Angle::from_radians(self.controller.setpoint) - context.data.heading)
            .wrapped_half()
            .as_radians();

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

        // log::debug!(
        //     "Control output: {:.2?} Error: {:.2?} Setpoint: {:.2?} heading: {:.2?}",
        //     output,
        //     error,
        //     self.controller.setpoint,
        //     context.pose.heading()
        // );

        // Apply the output as a voltage pair for rotation
        Some(crate::subsystems::drivetrain::DrivetrainPair {
            left: -output,
            right: output,
            units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::RPM,
        })
    }
}
