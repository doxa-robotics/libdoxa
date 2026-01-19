use nalgebra::Point2;

use crate::path_planner::Path;

/// A [`Path`] representing a linear path between two points.
#[derive(Debug)]
pub struct LinearPath {
    start: Point2<f64>,
    end: Point2<f64>,
}

impl Path for LinearPath {
    fn length(&self) -> f64 {
        nalgebra::distance(&self.start, &self.end)
    }

    fn length_until(&self, t: f64) -> f64 {
        self.length() * t
    }

    fn evaluate(&self, t: f64) -> Point2<f64> {
        Point2::new(
            self.start.x + (self.end.x - self.start.x) * t,
            self.start.y + (self.end.y - self.start.y) * t,
        )
    }

    fn evaluate_angle(&self, _t: f64) -> f64 {
        let delta_x = self.end.x - self.start.x;
        let delta_y = self.end.y - self.start.y;
        delta_y.atan2(delta_x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_path_length() {
        let path = LinearPath {
            start: Point2::new(0.0, 0.0),
            end: Point2::new(3.0, 4.0),
        };
        assert_eq!(path.length(), 5.0);
    }

    #[test]
    fn test_linear_path_evaluate() {
        let path = LinearPath {
            start: Point2::new(0.0, 0.0),
            end: Point2::new(10.0, 0.0),
        };
        let point = path.evaluate(0.5);
        assert_eq!(point, Point2::new(5.0, 0.0));
    }

    #[test]
    fn test_linear_path_evaluate_angle() {
        let path = LinearPath {
            start: Point2::new(0.0, 0.0),
            end: Point2::new(0.0, 10.0),
        };
        let angle = path.evaluate_angle(0.5);
        assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
    }
}
