use core::{
    fmt,
    iter::Sum,
    ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign},
};

use nalgebra::Vector2;
use vexide::float::Float as _;

/// A struct representing a 2D pose with x, y coordinates and a heading
/// in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub offset: Vector2<f64>,
    /// Heading in radians
    /// 0 is facing right, pi/2 is facing up, pi is facing left, and -pi/2 is facing down
    pub heading: f64,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            offset: Vector2::new(0.0, 0.0),
            heading: 0.0,
        }
    }
}

impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "({}, {}, \u{03B8} = {})",
            self.offset.x, self.offset.y, self.heading
        )
    }
}

impl From<(f64, f64, f64)> for Pose {
    fn from(coords: (f64, f64, f64)) -> Self {
        Pose {
            offset: Vector2::new(coords.0, coords.1),
            heading: coords.2,
        }
    }
}

impl From<(f64, f64)> for Pose {
    fn from(coords: (f64, f64)) -> Self {
        Pose {
            offset: Vector2::new(coords.0, coords.1),
            heading: 0.0,
        }
    }
}

impl From<Pose> for nalgebra::Point2<f64> {
    fn from(pose: Pose) -> Self {
        nalgebra::Point2::new(pose.offset.x, pose.offset.y)
    }
}

impl From<Pose> for nalgebra::Vector2<f64> {
    fn from(pose: Pose) -> Self {
        nalgebra::Vector2::new(pose.offset.x, pose.offset.y)
    }
}

impl Pose {
    /// Creates a new Pose with the given x, y coordinates and heading.
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Pose {
            offset: Vector2::new(x, y),
            heading,
        }
    }

    /// Creates a new Pose with the given x, y coordinates and a heading of 0.
    pub fn new_no_heading(x: f64, y: f64) -> Self {
        Pose {
            offset: Vector2::new(x, y),
            heading: 0.0,
        }
    }

    /// Creates a new Pose from a scalar
    pub fn from_scalar(scalar: f64) -> Self {
        Pose {
            offset: Vector2::new(scalar, scalar),
            heading: 0.0,
        }
    }

    /// Returns the x coordinate of the pose.
    pub fn x(&self) -> f64 {
        self.offset.x
    }

    /// Returns the y coordinate of the pose.
    pub fn y(&self) -> f64 {
        self.offset.y
    }

    /// Returns the heading of the pose in radians.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// Returns the distance between this pose and another pose.
    pub fn distance(&self, other: Pose) -> f64 {
        ((self.offset.x - other.offset.x).powi(2) + (self.offset.y - other.offset.y).powi(2)).sqrt()
    }

    /// Returns the angle between this pose and another pose in radians.
    pub fn angle_to(&self, other: Pose) -> f64 {
        (other.offset.y - self.offset.y).atan2(other.offset.x - self.offset.x)
    }
}

impl From<Pose> for vexide::devices::math::Point2<i16> {
    fn from(pose: Pose) -> vexide::devices::math::Point2<i16> {
        vexide::devices::math::Point2 {
            x: pose.offset.x as i16,
            y: pose.offset.y as i16,
        }
    }
}

impl Add for Pose {
    type Output = Self;

    fn add(self, other: Pose) -> Self {
        Self {
            offset: Vector2::new(
                self.offset.x + other.offset.x,
                self.offset.y + other.offset.y,
            ),
            heading: (self.heading + other.heading) / 2.0,
        }
    }
}

impl Sum for Pose {
    fn sum<I: Iterator<Item = Pose>>(iter: I) -> Self {
        iter.fold(Pose::default(), Add::add)
    }
}

impl Sub for Pose {
    type Output = Self;

    fn sub(self, other: Pose) -> Self {
        Self {
            offset: Vector2::new(
                self.offset.x - other.offset.x,
                self.offset.y - other.offset.y,
            ),
            heading: (self.heading + other.heading) / 2.0,
        }
    }
}

impl Mul<f64> for Pose {
    type Output = Pose;

    fn mul(self, scalar: f64) -> Self {
        Self {
            offset: Vector2::new(self.offset.x * scalar, self.offset.y * scalar),
            heading: self.heading,
        }
    }
}

impl Div<f64> for Pose {
    type Output = Pose;

    fn div(self, scalar: f64) -> Self {
        Self {
            offset: Vector2::new(self.offset.x / scalar, self.offset.y / scalar),
            heading: self.heading,
        }
    }
}

impl Neg for Pose {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            offset: Vector2::new(-self.offset.x, -self.offset.y),
            heading: -self.heading,
        }
    }
}

impl AddAssign for Pose {
    fn add_assign(&mut self, other: Pose) {
        self.offset.x += other.offset.x;
        self.offset.y += other.offset.y;
        self.heading = (self.heading + other.heading) / 2.0;
    }
}

impl SubAssign for Pose {
    fn sub_assign(&mut self, other: Pose) {
        self.offset.x -= other.offset.x;
        self.offset.y -= other.offset.y;
        self.heading = (self.heading + other.heading) / 2.0;
    }
}
