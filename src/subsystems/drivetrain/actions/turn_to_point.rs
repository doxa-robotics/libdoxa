use core::f64::consts::PI;

use nalgebra::Point2;

use super::{RotationAction, config::ActionConfig};

fn angle_between_points(from: &Point2<f64>, to: &Point2<f64>) -> f64 {
    (to.y - from.y).atan2(to.x - from.x)
}

/// An action that turns the robot to face a point.
///
/// This action uses a PID controller to turn the robot to face a point. A
/// setpoint is not needed, as the target point is used as the setpoint.
#[derive(Debug)]
pub struct TurnToPointAction {
    target: Point2<f64>,
    config: ActionConfig,
    reverse: bool,
    action: Option<RotationAction>,
}

impl TurnToPointAction {
    pub fn new(target: Point2<f64>, reverse: bool, config: ActionConfig) -> Self {
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
            let target_heading = angle_between_points(&context.data.offset, &self.target)
                + if self.reverse { PI } else { 0.0 };
            log::debug!(
                "{:?} -> {:?} ==> {:?}",
                context.data.offset,
                self.target,
                target_heading
            );
            self.action = Some(RotationAction::new(target_heading, self.config));
        }

        self.action.as_mut().unwrap().update(context)
    }
}
