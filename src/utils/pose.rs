/// A struct representing a 2D pose with x, y coordinates and a heading
/// in radians.
pub struct Pose {
    pub x: f32,
    pub y: f32,
    /// Heading in radians
    /// 0 is facing right, pi/2 is facing up, pi is facing left, and -pi/2 is facing down
    pub heading: f32,
}

impl From<(f32, f32, f32)> for Pose {
    fn from(coords: (f32, f32, f32)) -> Self {
        Pose {
            x: coords.0,
            y: coords.1,
            heading: coords.2,
        }
    }
}

impl Pose {
    /// Creates a new Pose with the given x, y coordinates and heading.
    pub fn new(x: f32, y: f32, heading: f32) -> Self {
        Pose { x, y, heading }
    }

    /// Returns the x coordinate of the pose.
    pub fn x(&self) -> f32 {
        self.x
    }

    /// Returns the y coordinate of the pose.
    pub fn y(&self) -> f32 {
        self.y
    }

    /// Returns the heading of the pose in radians.
    pub fn heading(&self) -> f32 {
        self.heading
    }
}
