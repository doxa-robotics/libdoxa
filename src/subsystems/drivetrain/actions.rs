use core::fmt::Debug;

use crate::utils::pose::Pose;

mod drive_to_point;
mod forward;
mod pure_pursuit;
mod rotation;
mod turn_to_point;
mod voltage;

/// A drivetrain action.
///
/// Actions are used to control the drivetrain. They are responsible for setting
/// the voltage of the motors.
///
/// If an action is done, it should return `None`.
pub trait Action: Debug {
    fn update(&mut self, context: ActionContext) -> Option<super::VoltagePair>;
}

#[derive(Debug, Clone, Copy)]
pub struct ActionContext {
    pub pose: Pose,
    pub left_offset: f64,
    pub right_offset: f64,
    pub left_velocity: f64,
    pub right_velocity: f64,
}

pub use drive_to_point::DriveToPointAction;
pub use forward::ForwardAction;
pub use pure_pursuit::PurePursuitAction;
pub use rotation::RotationAction;
pub use turn_to_point::TurnToPointAction;
pub use voltage::VoltageAction;
