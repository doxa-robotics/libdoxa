use core::fmt::Debug;

use crate::subsystems::tracking::TrackingData;

mod boomerang;
pub mod config;
// mod drive_to_point;
mod forward;
mod lazy;
// mod pure_pursuit;
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
    /// Updates the action and returns the desired drivetrain output.
    fn update(&mut self, context: ActionContext) -> Option<super::DrivetrainPair>;
}

#[derive(Debug, Clone, Copy)]
pub struct ActionContext {
    pub data: TrackingData,
}

pub use boomerang::BoomerangAction;
// pub use drive_to_point::DriveToPointAction;
pub use forward::ForwardAction;
pub use lazy::LazyAction;
// pub use pure_pursuit::PurePursuitAction;
pub use rotation::RotationAction;
pub use turn_to_point::TurnToPointAction;
pub use voltage::VoltageAction;
