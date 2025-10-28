//! Cubic parameteric path planner
//!
//! This module implements a path planner that generates a cubic polynomial
//! trajectory for the x and y coordinates of a robot. The trajectory is defined
//! by a start pose, an end pose, and velocity at the start and end of the
//! trajectory.
//!
//! A utility function is provided to combine two cubic trajectories into a
//! single trajectory. This is useful for creating a continuous path that
//! consists of multiple segments.
//!
//! The theory behind this can be found [here](http://geogebra.org/calculator/bkujghbu).

use nalgebra::Point2;
use vexide::math::Angle;

use super::Path;

#[rustfmt::skip]
const CURVE_FITTING_MATRIX: nalgebra::Matrix4<f64> = nalgebra::Matrix4::new(
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

#[derive(Debug, Clone, Copy)]
struct Cubic {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

impl Cubic {
    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { a, b, c, d }
    }

    pub fn evaluate(&self, t: f64) -> f64 {
        self.a * t.powi(3) + self.b * t.powi(2) + self.c * t + self.d
    }

    pub fn evaluate_derivative(&self, t: f64) -> f64 {
        3.0 * self.a * t.powi(2) + 2.0 * self.b * t + self.c
    }

    pub fn from_endpoints(
        start: f64,
        end: f64,
        start_derivative: f64,
        end_derivative: f64,
    ) -> Self {
        let vector = nalgebra::Vector4::new(start, end, start_derivative, end_derivative);
        let coeffs = CURVE_FITTING_MATRIX * vector;
        Cubic::new(coeffs.x, coeffs.y, coeffs.z, coeffs.w)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CubicParametricPath {
    x: Cubic,
    y: Cubic,
}

impl CubicParametricPath {
    /// Creates a new CubicParameterPath with the given start and end poses
    ///
    /// easing is a value from [0.0, infinity) that determines how "curvy" the
    /// path is.
    pub fn new(
        start_point: Point2<f64>,
        start_angle: Angle,
        start_easing: f64,
        end_point: Point2<f64>,
        end_angle: Angle,
        end_easing: f64,
    ) -> Self {
        let x = Cubic::from_endpoints(
            start_point.x,
            end_point.x,
            start_easing * start_angle.cos(),
            end_easing * end_angle.cos(),
        );
        let y = Cubic::from_endpoints(
            start_point.y,
            end_point.y,
            start_easing * start_angle.sin(),
            end_easing * end_angle.sin(),
        );

        Self { x, y }
    }

    pub fn debug_render(&self, display: &mut vexide::display::Display) {
        let dt = 0.01;
        let mut t = 0.0;
        let mut last_point = self.evaluate(0.0);
        while t <= 1.0 {
            let point = self.evaluate(t);
            display.fill(
                &vexide::display::Line::new(
                    vexide::math::Point2 {
                        x: (last_point.x) as i16,
                        y: (last_point.y) as i16,
                    },
                    vexide::math::Point2 {
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
    fn evaluate(&self, t: f64) -> Point2<f64> {
        let x = self.x.evaluate(t);
        let y = self.y.evaluate(t);
        Point2::new(x, y)
    }

    fn evaluate_angle(&self, t: f64) -> f64 {
        self.x
            .evaluate_derivative(t)
            .atan2(self.y.evaluate_derivative(t))
    }

    fn length_until(&self, max_t: f64) -> f64 {
        // TODO(@rh0820): #1 implement length calculation using calculus
        // for now, just using a small step size
        let dt = 0.001;
        let mut length = 0.0;
        let mut last_point = self.evaluate(0.0);
        let mut t = dt;
        if max_t >= 0.0 {
            while t <= max_t {
                let point = self.evaluate(t);
                length += nalgebra::distance(&last_point, &point);
                last_point = point;
                t += dt;
            }
        } else {
            while t >= max_t {
                let point = self.evaluate(t);
                length -= nalgebra::distance(&last_point, &point);
                last_point = point;
                t -= dt;
            }
        }
        length
    }
}
