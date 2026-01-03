use crate::utils::settling;
use pid::Pid;
use vexide::math::Angle;

use super::config::ActionConfig;

/// An action that rotates the drivetrain to a specific absolute heading.
///
/// This action uses a PID controller to rotate the drivetrain to a target
/// heading.
#[derive(Debug)]
pub struct RotationAction {
    controller: Pid<f64>,
    setpoint: f64,
    tolerances: settling::Tolerances,
}

impl RotationAction {
    pub fn new(target_radians: f64, config: ActionConfig) -> Self {
        Self {
            controller: config.turn_pid(0.0),
            setpoint: target_radians,
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
        let error = (Angle::from_radians(self.setpoint) - context.data.heading)
            .wrapped_half()
            .as_radians();
        log::trace!(
            "Rotation: {:.3} --> {:.3} (error: {:.3})",
            context.data.heading.as_radians(),
            self.setpoint,
            error
        );

        if self
            .tolerances
            .check(error, context.data.angular_velocity.as_radians())
        {
            return None;
        }

        let output = self.controller.next_control_output(error).output;

        // Apply the output as a voltage pair for rotation
        Some(crate::subsystems::drivetrain::DrivetrainPair::new_voltage(
            -output, output,
        ))
    }
}
