use crate::utils::{settling, vec2::Vec2};

use super::RotationAction;

/// An action that turns the robot to face a point.
///
/// This action uses a PID controller to turn the robot to face a point. A
/// setpoint is not needed, as the target point is used as the setpoint.
#[derive(Debug)]
pub struct TurnToPointAction {
    target: Vec2<f64>,
    controller: pid::Pid<f64>,
    tolerances: settling::Tolerances,
    action: Option<RotationAction>,
}

impl TurnToPointAction {
    pub fn new(
        target: Vec2<f64>,
        controller: pid::Pid<f64>,
        tolerances: settling::Tolerances,
    ) -> Self {
        Self {
            target,
            controller,
            tolerances,
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
            let target_heading = (self.target - context.offset).angle();
            self.controller.setpoint = target_heading;
            self.action = Some(RotationAction::new(self.controller, self.tolerances));
        }

        self.action.as_mut().unwrap().update(context)
    }
}
