use alloc::rc::Rc;
use core::cell::RefCell;
use vexide::{math::Angle, prelude::*};
use vexide_motorgroup::{MotorGroup, SharedMotors};

/// Trait for objects that have a rotational position.
pub trait HasRotation {
    /// Returns the position of the object.
    fn position(&self) -> Angle;
}

impl HasRotation for RotationSensor {
    fn position(&self) -> Angle {
        self.position().unwrap_or_default()
    }
}

impl HasRotation for Motor {
    fn position(&self) -> Angle {
        self.position().unwrap_or_default()
    }
}

impl HasRotation for MotorGroup {
    fn position(&self) -> Angle {
        self.position().unwrap_or_default()
    }
}

impl HasRotation for () {
    fn position(&self) -> Angle {
        Angle::default()
    }
}

impl HasRotation for SharedMotors {
    fn position(&self) -> Angle {
        self.position().unwrap_or_default()
    }
}

impl<T: HasRotation> HasRotation for Rc<RefCell<T>> {
    fn position(&self) -> Angle {
        self.borrow().position()
    }
}

/// Trait for objects that have a heading.
pub trait HasHeading {
    /// Returns the heading of the object. This value does not wrap around.
    fn heading(&self) -> Angle;
}

/// Trait for objects that have a wrapping heading.
pub trait HasWrappingHeading {
    /// Returns the heading of the object, wrapped to the range [0, 360).
    fn wrapping_heading(&self) -> Angle;
}

impl<T: HasHeading> HasWrappingHeading for T {
    fn wrapping_heading(&self) -> Angle {
        self.heading().wrapped_full()
    }
}

/// Struct to correct a heading that wraps around.
///
/// This will turn a heading that wraps around into an absolute heading.
pub struct WrappingHeadingCorrector<T: HasWrappingHeading> {
    last_heading: RefCell<Angle>,
    heading_offset: RefCell<Angle>,
    has_wrapping_heading: T,
}

impl<T: HasWrappingHeading> HasHeading for WrappingHeadingCorrector<T> {
    fn heading(&self) -> Angle {
        let current_heading = self.has_wrapping_heading.wrapping_heading();
        let last_heading = *self.last_heading.borrow();
        let heading_offset = *self.heading_offset.borrow();

        let delta = current_heading - last_heading;
        let delta = if delta > Angle::from_radians(core::f64::consts::PI) {
            delta - Angle::from_radians(2.0 * core::f64::consts::PI)
        } else if delta < -Angle::from_radians(core::f64::consts::PI) {
            delta + Angle::from_radians(2.0 * core::f64::consts::PI)
        } else {
            delta
        };

        *self.last_heading.borrow_mut() = current_heading;
        *self.heading_offset.borrow_mut() += delta;

        current_heading + heading_offset
    }
}

impl HasHeading for InertialSensor {
    fn heading(&self) -> Angle {
        self.rotation().unwrap_or_default()
    }
}

impl HasWrappingHeading for AdiGyroscope {
    fn wrapping_heading(&self) -> Angle {
        self.yaw().unwrap_or_default()
    }
}

impl<T: HasHeading> HasHeading for Rc<RefCell<T>> {
    fn heading(&self) -> Angle {
        self.try_borrow().map_or(Angle::default(), |f| f.heading())
    }
}
