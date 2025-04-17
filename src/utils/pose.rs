use vexide::float::Float as _;

/// A struct representing a 2D pose with x, y coordinates and a heading
/// in radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    /// Heading in radians
    /// 0 is facing right, pi/2 is facing up, pi is facing left, and -pi/2 is facing down
    pub heading: f64,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            heading: 0.0,
        }
    }
}

impl From<(f64, f64, f64)> for Pose {
    fn from(coords: (f64, f64, f64)) -> Self {
        Pose {
            x: coords.0,
            y: coords.1,
            heading: coords.2,
        }
    }
}

impl Pose {
    /// Creates a new Pose with the given x, y coordinates and heading.
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Pose { x, y, heading }
    }

    /// Returns the x coordinate of the pose.
    pub fn x(&self) -> f64 {
        self.x
    }

    /// Returns the y coordinate of the pose.
    pub fn y(&self) -> f64 {
        self.y
    }

    /// Returns the heading of the pose in radians.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    /// Returns the distance between this pose and another pose.
    pub fn distance(&self, other: Pose) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    /// Returns the angle between this pose and another pose in radians.
    pub fn angle_to(&self, other: Pose) -> f64 {
        (other.y - self.y).atan2(other.x - self.x)
    }
}

impl From<Pose> for vexide::devices::math::Point2<i16> {
    fn from(pose: Pose) -> vexide::devices::math::Point2<i16> {
        vexide::devices::math::Point2 {
            x: pose.x as i16,
            y: pose.y as i16,
        }
    }
}
