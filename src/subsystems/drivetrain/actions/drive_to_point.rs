use super::{forward::ForwardAction, turn_to_point::TurnToPointAction, Action, ActionContext};
use crate::subsystems::drivetrain::VoltagePair;
use crate::utils::pose::Pose;
use crate::utils::settling;

/// An action that drives the robot to a specific point.
///
/// This action first turns the robot to face the target point, and then drives
/// forward to reach the target point. This does not take into account the
/// heading of the target pose.
#[derive(Debug)]
pub struct DriveToPointAction {
    target: Pose,
    state: DriveToPointState,
    turn_controller: pid::Pid<f64>,
    forward_controller: pid::Pid<f64>,
    turn_tolerances: settling::Tolerances,
    forward_tolerances: settling::Tolerances,
}

#[derive(Debug)]
enum DriveToPointState {
    NotStarted,
    Turning(TurnToPointAction),
    Driving(ForwardAction),
    Done,
}

impl DriveToPointAction {
    pub fn new(
        target: Pose,
        turn_controller: pid::Pid<f64>,
        forward_controller: pid::Pid<f64>,
        turn_tolerances: settling::Tolerances,
        forward_tolerances: settling::Tolerances,
    ) -> Self {
        Self {
            target,
            state: DriveToPointState::NotStarted,
            turn_controller,
            forward_controller,
            turn_tolerances,
            forward_tolerances,
        }
    }
}

impl Action for DriveToPointAction {
    fn update(&mut self, context: ActionContext) -> Option<VoltagePair> {
        match &mut self.state {
            DriveToPointState::NotStarted => {
                // Transition to the turning state
                let target_heading = context.pose.angle_to(self.target);
                self.turn_controller.setpoint = target_heading;
                self.state = DriveToPointState::Turning(TurnToPointAction::new(
                    self.target,
                    self.turn_controller,
                    self.turn_tolerances,
                ));
                self.update(context)
            }
            DriveToPointState::Turning(turn_action) => {
                if let Some(voltage) = turn_action.update(context) {
                    return Some(voltage);
                }
                // Transition to driving action
                self.forward_controller.setpoint = self.target.distance(context.pose);
                log::debug!(
                    "DriveToPointAction: Forward setpoint: {}",
                    self.forward_controller.setpoint
                );
                self.state = DriveToPointState::Driving(ForwardAction::new(
                    self.forward_controller,
                    self.forward_tolerances,
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
