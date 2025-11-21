use core::f64::consts::PI;

use crate::utils::settling;
use pid::Pid;

use super::config::ActionConfig;

/// An action that rotates the drivetrain to a specific absolute heading.
///
/// This action uses a PID controller to rotate the drivetrain to a target
/// heading.
#[derive(Debug)]
pub struct RotationAction {
    controller: Pid<f64>,
    tolerances: settling::Tolerances,
}

impl RotationAction {
    pub fn new(target_radians: f64, config: ActionConfig) -> Self {
        Self {
            controller: config.turn_pid(target_radians),
            tolerances: config.turn_tolerances(),
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
        let error = (self.controller.setpoint - context.data.heading.as_radians() + PI)
            .rem_euclid(2.0 * PI)
            - PI;

        if self
            .tolerances
            .check(error, context.data.angular_velocity.as_radians())
        {
            return None;
        }

        let output = self
            .controller
            .next_control_output(context.data.heading.as_radians())
            .output;

        // Apply the output as a voltage pair for rotation
        Some(crate::subsystems::drivetrain::DrivetrainPair {
            left: -output,
            right: output,
            units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::RPM,
        })
    }
}
