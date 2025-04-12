use crate::utils::pose::Pose;

pub mod cubic_parametric;

pub trait Path {
    fn length_until(&self, t: f32) -> f32;
    fn evaluate(&self, t: f32) -> Pose;

    fn length(&self) -> f32 {
        self.length_until(1.0)
    }
    fn point_on_radius(&self, pose: Pose, radius: f32, initial_t: Option<f32>) -> Option<f32> {
        let dt = 0.01;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f32::MAX;
        // let mut last_distance = self.evaluate(t).distance(&pose);
        while t <= 1.0 {
            let current_point = self.evaluate(t);
            let distance = current_point.distance(&pose);
            if (distance - radius).abs() < closest_distance {
                closest_distance = (distance - radius).abs();
                closest_t = t;
            }
            // Seems to not work
            // if distance > last_distance {
            //     break;
            // }
            // last_distance = distance;
            t += dt;
        }
        if closest_distance < 3.0 {
            Some(closest_t)
        } else {
            None
        }
    }
    fn closest_point(&self, pose: Pose, initial_t: Option<f32>) -> f32 {
        let dt = 0.01;
        let mut t = initial_t.unwrap_or(0.0);
        let mut closest_t = t;
        let mut closest_distance = f32::MAX;
        let mut last_distance = self.evaluate(t).distance(&pose);
        while t <= 1.0 {
            let current_point = self.evaluate(t);
            let distance = current_point.distance(&pose);
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
        while t >= 0.0 {
            let current_point = self.evaluate(t);
            let distance = current_point.distance(&pose);
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
