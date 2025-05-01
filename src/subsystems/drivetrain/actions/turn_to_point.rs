use core::f64::consts::PI;

use crate::utils::pose::Pose;

use super::{config::ActionConfig, RotationAction};

/// An action that turns the robot to face a point.
///
/// This action uses a PID controller to turn the robot to face a point. A
/// setpoint is not needed, as the target point is used as the setpoint.
#[derive(Debug)]
pub struct TurnToPointAction {
    target: Pose,
    config: ActionConfig,
    reverse: bool,
    action: Option<RotationAction>,
}

impl TurnToPointAction {
    pub fn new(target: Pose, reverse: bool, config: ActionConfig) -> Self {
        Self {
            target,
            config,
            action: None,
            reverse,
        }
    }
}

impl super::Action for TurnToPointAction {
    fn update(
        &mut self,
        context: super::ActionContext,
    ) -> Option<crate::subsystems::drivetrain::DrivetrainPair> {
        if self.action.is_none() {
            let target_heading =
                context.pose.angle_to(self.target) + if self.reverse { PI } else { 0.0 };
            log::debug!(
                "{:?} -> {:?} ==> {:?}",
                context.pose,
                self.target,
                target_heading
            );
            self.action = Some(RotationAction::new(target_heading, self.config));
        }

        self.action.as_mut().unwrap().update(context)
    }
}
