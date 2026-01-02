use core::fmt::Debug;

use nalgebra::Point2;

pub mod compound;
pub mod cubic_parametric;

pub trait Path: Debug {
    /// Returns the length of the path from t=0 to t=`t`. This is calculated as
    /// the integral of the path's derivative (arc length) from 0 to `t`.
    fn length_until(&self, t: f64) -> f64;

    /// Evaluates the path at a given t value in [0, 1] to produce a 2D point.
    fn evaluate(&self, t: f64) -> Point2<f64>;

    /// Evaluates the angle of the path at a given t value in [0, 1].
    ///
    /// This corresponds to the tangent of the path at that point. The default
    /// implementation approximates this using the implementation of `evaluate`.
    fn evaluate_angle(&self, t: f64) -> f64 {
        let dt = 0.0001;
        let p1 = self.evaluate(t);
        let p2 = self.evaluate(t + dt);
        (p2.y - p1.y).atan2(p2.x - p1.x)
    }

    /// Returns the total length of the path.
    ///
    /// Typically, unless there is a more efficient implementation, the default
    /// implementation of `length_until(1.0)` can be used.
    fn length(&self) -> f64 {
        self.length_until(1.0)
    }

    /// Finds a point on the path that is approximately `radius` distance from
    /// the given `point`.
    fn point_on_radius(
        &self,
        point: Point2<f64>,
        radius: f64,
        initial_t: Option<f64>,
    ) -> Option<f64> {
        let dt = 0.001;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f64::MAX;
        // let mut last_distance = self.evaluate(t).distance(&pose);
        while t <= 1.0 {
            let current_point = self.evaluate(t);
            let distance = nalgebra::distance(&current_point, &point);
            if (distance - radius).abs() < closest_distance {
                closest_distance = (distance - radius).abs();
                closest_t = t;
            }
            // Seems to not work
            // TODO: we don't want to skip a path segment so we should check
            // whether the last point is much farther along the path than the previous
            // if distance > last_distance {
            //     break;
            // }
            // last_distance = distance;
            t += dt;
        }
        if closest_distance < 3.0 {
            Some(closest_t)
        } else {
            log::error!(
                "Path: No point on path found within radius {} of point {:?}. Closest point was {} away",
                radius,
                point,
                closest_distance
            );
            None
        }
    }

    /// Finds the closest point on the path to the given `point`.
    ///
    /// An optional `initial_t` can be provided to start the search from a
    /// specific point on the path. An optional `overshoot` can be provided to
    /// allow searching slightly beyond the [0, 1] range.
    fn closest_point(
        &self,
        point: Point2<f64>,
        initial_t: Option<f64>,
        overshoot: Option<f64>,
    ) -> f64 {
        let dt = 0.01;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f64::MAX;
        let mut last_distance = nalgebra::distance(&self.evaluate(t), &point);
        let overshoot = overshoot.unwrap_or(0.0);
        while t <= 1.0 + overshoot {
            let current_point = self.evaluate(t);
            let distance = nalgebra::distance(&current_point, &point);
            if distance < closest_distance {
                closest_distance = distance;
                closest_t = t;
            }
            if distance > last_distance {
                break;
            }
            last_distance = distance;
            t += dt;
        }
        // Do a reverse search
        t = initial_t.unwrap_or(0.0);
        while t >= 0.0 - overshoot {
            let current_point = self.evaluate(t);
            let distance = nalgebra::distance(&current_point, &point);
            if distance < closest_distance {
                closest_distance = distance;
                closest_t = t;
            }
            if distance > last_distance {
                break;
            }
            last_distance = distance;
            t -= dt;
        }
        closest_t
    }
}
