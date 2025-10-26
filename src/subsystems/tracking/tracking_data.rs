use nalgebra::{Point2, Vector2};
use vexide::math::Angle;

/// Struct representing tracking data from the tracking subsystem, containing
/// the current pose and its derivative.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TrackingData {
    pub offset: Point2<f64>,
    /// The normalized heading angle in radians.
    pub heading: Angle,

    pub velocity: Vector2<f64>,
    pub angular_velocity: Angle,

    pub timestamp: Option<std::time::Instant>,
}

impl TrackingData {
    /// Creates a new `TrackingData` instance based on the current one,
    /// computing velocity and angular velocity from the change in pose over
    /// time.
    ///
    /// If the old timestamp is `None`, `velocity` and `angular_velocity` will
    /// be set to zero.
    pub(crate) fn advance(&self, new_offset: Point2<f64>, new_heading: Angle) -> Self {
        let new_heading = new_heading.wrapped(Angle::ZERO..Angle::FULL_TURN);
        if let Some(old_timestamp) = self.timestamp {
            let now = std::time::Instant::now();
            let dt = now.duration_since(old_timestamp).as_secs_f64();
            let velocity = (new_offset - self.offset) / dt;
            let angular_velocity = (new_heading - self.heading) / dt;
            Self {
                offset: new_offset,
                heading: new_heading,
                velocity,
                angular_velocity,
                timestamp: Some(now),
            }
        } else {
            Self {
                offset: new_offset,
                heading: new_heading,
                velocity: Vector2::default(),
                angular_velocity: Angle::default(),
                timestamp: Some(std::time::Instant::now()),
            }
        }
    }

    /// Computes the linear velocity in the direction of movement (the heading).
    ///
    /// This is typically the signed magnitude of the velocity vector, but may
    /// be computed differently if the robot is pushed, e.g., sideways.
    ///
    /// Mathematically, this is the dot product of the velocity vector and the
    /// unit vector in the direction of the heading. See:
    /// https://www.khanacademy.org/math/multivariable-calculus/thinking-about-multivariable-function/x786f2022:vectors-and-matrices/a/dot-products-mvc
    pub fn linear_velocity(&self) -> f64 {
        let heading_vector = Vector2::new(self.heading.cos(), self.heading.sin());
        self.velocity.dot(&heading_vector)
    }
}
