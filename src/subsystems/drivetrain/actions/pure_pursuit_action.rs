use pid::Pid;
use vexide::time::Instant;

use crate::{
    path_planner::Path,
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

#[derive(Debug)]
pub struct PurePursuitAction<T: Path> {
    path: T,
    path_total: f32,
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    last_t: f32,
    last_path_distance: f32,
    target_point: Pose,
    lookahead: f32,
    last_update: Option<Instant>,
    settled: bool,
    end_pose: Pose,
    max_voltage: f64,
}

impl<T: Path> PurePursuitAction<T> {
    pub fn new(
        path: T,
        rotational_pid: Pid<f64>,
        mut linear_pid: Pid<f64>,
        linear_tolerances: Tolerances,
        lookahead: f32,
        max_voltage: f64,
    ) -> Self {
        linear_pid.setpoint(0.0);
        Self {
            end_pose: path.evaluate(1.0),
            path_total: path.length(),
            path,
            rotational_pid,
            linear_pid,
            linear_tolerances,
            last_t: 0.0,
            last_path_distance: 0.0,
            target_point: Pose::default(),
            lookahead,
            last_update: None,
            settled: false,
            max_voltage,
        }
    }
}

impl<T: Path> super::Action for PurePursuitAction<T> {
    fn update(&mut self, context: super::ActionContext) -> Option<VoltagePair> {
        // If we are settled, we don't need to do anything
        if self.settled {
            return None;
        }
        // Convert the current pose to a Pose. This should be removed after
        // `context` is updated to use Pose directly
        let current_pose = Pose::new(
            context.offset.x as f32,
            context.offset.y as f32,
            context.heading as f32,
        );
        // Find the closest point on the path to the current pose
        let current_t = self
            .path
            .closest_point(current_pose, Some(self.last_t), Some(0.1));
        if current_t != self.last_t {
            self.last_update = Some(Instant::now());
        }
        // Find how far along the path we are
        let path_distance = self.path.length_until(current_t);
        // Calculate the distance and velocity to the end of the path
        let error = self.path_total - path_distance;
        let velocity = (path_distance - self.last_path_distance)
            / self
                .last_update
                .unwrap_or(Instant::now())
                .elapsed()
                .as_secs_f32()
            // Avoid division by zero
            + 0.0001;
        // Are we there yet?
        if self.linear_tolerances.check(error as f64, velocity as f64) {
            self.settled = true;
            return None;
        }
        self.last_path_distance = path_distance;
        self.last_t = current_t;

        // Calculate the linear part of the differential drive
        // We simply use a PID controller while plugging in the error along the path
        let linear_voltage = self.linear_pid.next_control_output(error as f64).output;

        // Calculate the rotational part of the differential drive
        // With pure pursuit, we find the intersection of the path and a circle
        // with radius equal to the lookahead distance
        // Calculate the angle to the target point
        if let Some(target_t) =
            self.path
                .point_on_radius(current_pose, self.lookahead, Some(current_t))
        {
            self.target_point = self.path.evaluate(target_t);
        } else if self.target_point.distance(current_pose) <= self.lookahead {
            // If we can't find a target point but we're within the lookahead
            // distance, we can just use the end of the path
            self.target_point = self.end_pose;
        } else {
            // We can't find a target point and we've strayed too far from the path
            // Just use the last target point
        }
        // Calculate the angle to the target point
        let angle_to_target = current_pose.angle_to(self.target_point) + current_pose.heading();

        self.rotational_pid.setpoint(angle_to_target as f64);
        let rotational_voltage = self
            .rotational_pid
            .next_control_output(current_pose.heading() as f64)
            .output;

        log::debug!(
            "Calculated voltages - Left: {}, Right: {}",
            linear_voltage - rotational_voltage,
            linear_voltage + rotational_voltage
        );

        Some(
            VoltagePair {
                left: linear_voltage - rotational_voltage,
                right: linear_voltage + rotational_voltage,
            }
            .max_voltage(self.max_voltage),
        )
    }
}
