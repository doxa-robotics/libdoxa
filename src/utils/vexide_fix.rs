use std::f64::consts::{PI, TAU};

use vexide::math::Angle;

pub trait VexideWrappedHalfFixExt {
    /// Returns the angle wrapped to the range [-180, 180).
    fn wrapped_half_fixed(&self) -> Self;
}

impl VexideWrappedHalfFixExt for Angle {
    fn wrapped_half_fixed(&self) -> Self {
        Self::from_radians((self.as_radians() + PI).rem_euclid(TAU) - PI)
    }
}
