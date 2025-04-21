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
    action: Option<RotationAction>,
}

impl TurnToPointAction {
    pub fn new(target: Pose, config: ActionConfig) -> Self {
        Self {
            target,
            config,
            action: None,
        }
    }
}

impl super::Action for TurnToPointAction {
    fn update(
        &mut self,
        context: super::ActionContext,
    ) -> Option<crate::subsystems::drivetrain::VoltagePair> {
        if self.action.is_none() {
            let target_heading = context.pose.angle_to(self.target);
            self.action = Some(RotationAction::new(target_heading, self.config));
        }

        self.action.as_mut().unwrap().update(context)
    }
}
