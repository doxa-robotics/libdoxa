use alloc::rc::Rc;
use core::cell::RefCell;
use vexide::{
    float::Float,
    prelude::{AdiGyroscope, InertialSensor, Motor, Position, RotationSensor},
};
use vexide_motorgroup::MotorGroup;

/// Trait for objects that have a rotational position.
pub trait HasRotation {
    /// Returns the position of the object.
    fn position(&self) -> Position;
}

impl HasRotation for RotationSensor {
    fn position(&self) -> Position {
        self.position().unwrap_or_default()
    }
}

impl HasRotation for Motor {
    fn position(&self) -> Position {
        self.position().unwrap_or_default()
    }
}

impl HasRotation for MotorGroup {
    fn position(&self) -> Position {
        self.position().unwrap_or_default()
    }
}

impl<T: HasRotation> HasRotation for Rc<RefCell<T>> {
    fn position(&self) -> Position {
        self.borrow().position()
    }
}

/// Trait for objects that have a heading in degrees.
pub trait HasHeading {
    /// Returns the heading of the object in degrees. This value does not wrap around.
    fn heading(&self) -> f64;
}

/// Trait for objects that have a wrapping heading in degrees.
pub trait HasWrappingHeading {
    /// Returns the heading of the object in degrees, wrapped to the range [0, 360).
    fn wrapping_heading(&self) -> f64;
}

impl<T: HasHeading> HasWrappingHeading for T {
    fn wrapping_heading(&self) -> f64 {
        self.heading().rem_euclid(360.0)
    }
}

/// Struct to correct a heading that wraps around.
///
/// This will turn a heading that wraps around into an absolute heading.
pub struct WrappingHeadingCorrector<T: HasWrappingHeading> {
    last_heading: RefCell<f64>,
    heading_offset: RefCell<f64>,
    has_wrapping_heading: T,
}

impl<T: HasWrappingHeading> HasHeading for WrappingHeadingCorrector<T> {
    fn heading(&self) -> f64 {
        let current_heading = self.has_wrapping_heading.wrapping_heading();
        let last_heading = *self.last_heading.borrow();
        let heading_offset = *self.heading_offset.borrow();

        let delta = current_heading - last_heading;
        let delta = if delta > 180.0 {
            delta - 360.0
        } else if delta < -180.0 {
            delta + 360.0
        } else {
            delta
        };

        *self.last_heading.borrow_mut() = current_heading;
        *self.heading_offset.borrow_mut() += delta;

        current_heading + heading_offset
    }
}

impl HasHeading for InertialSensor {
    fn heading(&self) -> f64 {
        self.rotation().unwrap_or_default()
    }
}

impl HasWrappingHeading for AdiGyroscope {
    fn wrapping_heading(&self) -> f64 {
        self.yaw().unwrap_or_default().as_degrees()
    }
}

impl<T: HasHeading> HasHeading for Rc<RefCell<T>> {
    fn heading(&self) -> f64 {
        self.borrow().heading()
    }
}
