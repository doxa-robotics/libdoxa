use core::f64::consts::PI;

use super::config::ActionConfig;
use super::{forward::ForwardAction, turn_to_point::TurnToPointAction, Action, ActionContext};
use crate::subsystems::drivetrain::VoltagePair;
use crate::utils::pose::Pose;

/// An action that drives the robot to a specific point.
///
/// This action first turns the robot to face the target point, and then drives
/// forward to reach the target point. This does not take into account the
/// heading of the target pose.
#[derive(Debug)]
pub struct DriveToPointAction {
    target: Pose,
    reverse: bool,
    state: DriveToPointState,
    turn_controller: pid::Pid<f64>,
    config: super::config::ActionConfig,
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
enum DriveToPointState {
    NotStarted,
    Turning(TurnToPointAction),
    Driving(ForwardAction),
    Done,
}

impl DriveToPointAction {
    pub fn new(target: Pose, reverse: bool, config: ActionConfig) -> Self {
        Self {
            target,
            reverse,
            state: DriveToPointState::NotStarted,
            turn_controller: config.turn_pid(0.0),
            config,
        }
    }
}

impl Action for DriveToPointAction {
    fn update(&mut self, context: ActionContext) -> Option<VoltagePair> {
        match &mut self.state {
            DriveToPointState::NotStarted => {
                // Transition to the turning state
                self.state = DriveToPointState::Turning(TurnToPointAction::new(
                    self.target,
                    self.reverse,
                    self.config,
                ));
                self.update(context)
            }
            DriveToPointState::Turning(turn_action) => {
                if let Some(voltage) = turn_action.update(context) {
                    return Some(voltage);
                }
                // Transition to driving action
                self.state = DriveToPointState::Driving(ForwardAction::new(
                    self.target.distance(context.pose) * if self.reverse { -1.0 } else { 1.0 },
                    self.config,
                ));
                self.update(context)
            }
            DriveToPointState::Driving(forward_action) => {
                if let Some(voltage) = forward_action.update(context) {
                    return Some(voltage);
                }
                // Mark as done
                self.state = DriveToPointState::Done;
                None
            }
            DriveToPointState::Done => None,
        }
    }
}
