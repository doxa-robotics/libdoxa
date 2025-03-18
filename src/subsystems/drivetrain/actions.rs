use crate::utils::vec2::Vec2;

mod forward;
mod voltage;

/// A drivetrain action.
///
/// Actions are used to control the drivetrain. They are responsible for setting
/// the voltage of the motors.
///
/// If an action is done, it should return `None`.
pub trait Action {
    fn update(&mut self, context: ActionContext) -> Option<super::VoltagePair>;
}

#[derive(Debug, Clone, Copy)]
pub struct ActionContext {
    pub offset: Vec2<f64>,
    pub left_offset: f64,
    pub right_offset: f64,
    pub left_velocity: f64,
    pub right_velocity: f64,
}

pub use forward::ForwardAction;
pub use voltage::VoltageAction;
