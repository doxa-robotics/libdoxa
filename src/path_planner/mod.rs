use core::fmt::Debug;

use crate::utils::pose::Pose;

pub mod compound;
pub mod cubic_parametric;

pub trait Path: Debug {
    fn length_until(&self, t: f64) -> f64;
    fn evaluate(&self, t: f64) -> Pose;

    fn length(&self) -> f64 {
        self.length_until(1.0)
    }
    fn point_on_radius(&self, pose: Pose, radius: f64, initial_t: Option<f64>) -> Option<f64> {
        let dt = 0.001;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f64::MAX;
        // let mut last_distance = self.evaluate(t).distance(&pose);
        while t <= 1.0 {
            let current_point = self.evaluate(t);
            let distance = current_point.distance(pose);
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
                "No point on path found within radius {} of pose {:?}. Closest point was {} away",
                radius,
                pose,
                closest_distance
            );
            None
        }
    }
    fn closest_point(&self, pose: Pose, initial_t: Option<f64>, overshoot: Option<f64>) -> f64 {
        let dt = 0.01;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f64::MAX;
        let mut last_distance = self.evaluate(t).distance(pose);
        let overshoot = overshoot.unwrap_or(0.0);
        while t <= 1.0 + overshoot {
            let current_point = self.evaluate(t);
            let distance = current_point.distance(pose);
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
            let distance = current_point.distance(pose);
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
