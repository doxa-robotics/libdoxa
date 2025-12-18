use nalgebra::Point2;

use super::BoomerangAction;
use super::config::ActionConfig;
use super::{Action, ActionContext, turn_to_point::TurnToPointAction};
use crate::subsystems::drivetrain::DrivetrainPair;

/// An action that drives the robot to a specific point.
///
/// This action first turns the robot to face the target point, and then drives
/// forward to reach the target point. This does not take into account the
/// heading of the target pose.
#[derive(Debug)]
pub struct DriveToPointAction {
    target: Point2<f64>,
    reverse: bool,
    state: DriveToPointState,
    config: super::config::ActionConfig,
    turn_config: super::config::ActionConfig,
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
enum DriveToPointState {
    NotStarted,
    Turning(TurnToPointAction),
    Driving(BoomerangAction),
    Done,
}

impl DriveToPointAction {
    pub fn new(target: Point2<f64>, config: ActionConfig) -> Self {
        Self {
            target,
            reverse: false,
            state: DriveToPointState::NotStarted,
            config,
            turn_config: config,
        }
    }

    pub fn with_turn_config(mut self, turn_config: ActionConfig) -> Self {
        self.turn_config = turn_config;
        self
    }

    pub fn with_reverse(mut self, reverse: bool) -> Self {
        self.reverse = reverse;
        self
    }
}

impl Action for DriveToPointAction {
    fn update(&mut self, context: ActionContext) -> Option<DrivetrainPair> {
        match &mut self.state {
            DriveToPointState::NotStarted => {
                // Transition to the turning state
                self.state = DriveToPointState::Turning(TurnToPointAction::new(
                    self.target,
                    self.reverse,
                    self.turn_config,
                ));
                self.update(context)
            }
            DriveToPointState::Turning(turn_action) => {
                if let Some(voltage) = turn_action.update(context) {
                    return Some(voltage);
                }
                // Transition to driving action
                self.state = DriveToPointState::Driving(BoomerangAction::new(
                    self.target,
                    context.data.heading,
                    // + if self.reverse { Angle::HALF_TURN
                    // } else {
                    //     Angle::ZERO
                    // },
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
