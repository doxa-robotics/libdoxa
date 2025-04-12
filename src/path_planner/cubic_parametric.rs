//! Cubic parameteric path planner
//!
//! This module implements a path planner that generates a cubic polynomial trajectory
//! for the x and y coordinates of a robot. The trajectory is defined by a start pose,
//! an end pose, and velocity at the start and end of the trajectory.
//!
//! A utility function is provided to combine two cubic trajectories into a single
//! trajectory. This is useful for creating a continuous path that consists of multiple
//! segments.
//!
//! The theory behind this can be found [here](http://geogebra.org/calculator/bkujghbu).

use vexide::{io::println, prelude::Float as _};

use crate::utils::pose::Pose;

use super::Path;

#[rustfmt::skip]
const CURVE_FITTING_MATRIX: nalgebra::Matrix4<f32> = nalgebra::Matrix4::new(
    2.0, -2.0, 1.0, 1.0,
    -3.0, 3.0, -2.0, -1.0,
    0.0, 0.0, 1.0, 0.0,
    1.0, 0.0, 0.0, 0.0
    // Inverse of
    // 0.0, 0.0, 0.0, 1.0,
    // 1.0, 1.0, 1.0, 1.0,
    // 0.0, 0.0, 1.0, 0.0,
    // 3.0, 2.0, 1.0, 0.0
);

struct Cubic {
    pub a: f32,
    pub b: f32,
    pub c: f32,
    pub d: f32,
}

impl Cubic {
    pub fn new(a: f32, b: f32, c: f32, d: f32) -> Self {
        Self { a, b, c, d }
    }

    pub fn evaluate(&self, t: f32) -> f32 {
        self.a * t.powi(3) + self.b * t.powi(2) + self.c * t + self.d
    }

    pub fn evaluate_derivative(&self, t: f32) -> f32 {
        3.0 * self.a * t.powi(2) + 2.0 * self.b * t + self.c
    }

    pub fn evaluate_second_derivative(&self, t: f32) -> f32 {
        6.0 * self.a * t + 2.0 * self.b
    }

    pub fn from_endpoints(
        start: f32,
        end: f32,
        start_derivative: f32,
        end_derivative: f32,
    ) -> Self {
        let vector = nalgebra::Vector4::new(start, end, start_derivative, end_derivative);
        let coeffs = CURVE_FITTING_MATRIX * vector;
        Cubic::new(coeffs.x, coeffs.y, coeffs.z, coeffs.w)
    }
}

pub struct CubicParametricPath {
    x: Cubic,
    y: Cubic,
}

impl CubicParametricPath {
    /// Creates a new CubicParameterPath with the given start and end poses
    ///
    /// easing is a value from [0.0, infinity) that determines how "curvy" the path is.
    pub fn new(start_pose: Pose, start_easing: f32, end_pose: Pose, end_easing: f32) -> Self {
        let x = Cubic::from_endpoints(
            start_pose.x(),
            end_pose.x(),
            start_easing * start_pose.heading().cos(),
            end_easing * end_pose.heading().cos(),
        );
        let y = Cubic::from_endpoints(
            start_pose.y(),
            end_pose.y(),
            start_easing * start_pose.heading().sin(),
            end_easing * end_pose.heading().sin(),
        );

        Self { x, y }
    }

    pub fn debug_render(&self, display: &mut vexide::devices::display::Display) {
        let dt = 0.01;
        let mut t = 0.0;
        let mut last_point = self.evaluate(0.0);
        while t <= 1.0 {
            let point = self.evaluate(t);
            display.fill(
                &vexide::devices::display::Line::new(
                    vexide::devices::math::Point2 {
                        x: (last_point.x) as i16,
                        y: (last_point.y) as i16,
                    },
                    vexide::devices::math::Point2 {
                        x: (point.x) as i16,
                        y: (point.y) as i16,
                    },
                ),
                (255, 255, 255),
            );
            last_point = point;
            t += dt;
        }
    }
}

impl Path for CubicParametricPath {
    fn evaluate(&self, t: f32) -> Pose {
        let x = self.x.evaluate(t);
        let y = self.y.evaluate(t);
        let heading = self
            .x
            .evaluate_derivative(t)
            .atan2(self.y.evaluate_derivative(t));
        Pose::new(x, y, heading)
    }

    fn length_until(&self, t: f32) -> f32 {
        // TODO(@rh0820): #1 implement length calculation using calculus
        // for now, just using a small step size
        let dt = 0.001;
        let mut length = 0.0;
        let mut last_point = self.evaluate(0.0);
        let mut t = dt;
        while t <= 1.0 {
            let point = self.evaluate(t);
            length += last_point.distance(&point);
            last_point = point;
            t += dt;
        }
        length
    }
}
