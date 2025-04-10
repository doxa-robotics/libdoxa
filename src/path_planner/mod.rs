use crate::utils::pose::Pose;

pub mod cubic_parametric;

pub trait Path {
    fn length(&self) -> f32;
    fn evaluate(&self, t: f32) -> Pose;
}
