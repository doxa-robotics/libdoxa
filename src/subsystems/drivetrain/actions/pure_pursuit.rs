use core::f64::consts::PI;

use pid::Pid;

use crate::{
    path_planner::Path,
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

#[derive(Debug)]
pub struct PurePursuitAction<T: Path> {
    path: T,
    path_total: f64,
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    last_t: f64,
    last_path_distance: f64,
    target_point: Pose,
    lookahead: f64,
    settled: bool,
    end_pose: Pose,
    max_voltage: f64,
    final_seeking: bool,
    seeking_pid: Pid<f64>,
}

impl<T: Path> PurePursuitAction<T> {
    pub fn new(
        path: T,
        rotational_pid: Pid<f64>,
        mut linear_pid: Pid<f64>,
        seeking_pid: Pid<f64>,
        linear_tolerances: Tolerances,
        lookahead: f64,
        max_voltage: f64,
    ) -> Self {
        let path_total = path.length();
        linear_pid.setpoint(path_total);
        Self {
            end_pose: path.evaluate(1.0),
            path_total,
            target_point: path.evaluate(0.0),
            path,
            rotational_pid,
            linear_pid,
            seeking_pid,
            linear_tolerances,
            last_t: 0.0,
            last_path_distance: 0.0,
            lookahead,
            settled: false,
            max_voltage,
            final_seeking: false,
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
        if self.final_seeking {
            let distance = self.target_point.distance(context.pose);
            if distance > self.lookahead {
                self.final_seeking = false;
            }
            self.linear_pid.setpoint(distance);
            let mut linear_voltage = self.linear_pid.next_control_output(distance).output;
            let mut angle_to_target = context.pose.angle_to(self.end_pose);
            log::debug!("angle_to_target: {}", angle_to_target);
            // Adjust angle_to_target to be closest to the context.pose.heading()
            while angle_to_target - context.pose.heading() > PI {
                angle_to_target -= 2.0 * PI;
            }
            while angle_to_target - context.pose.heading() < -PI {
                angle_to_target += 2.0 * PI;
            }
            if angle_to_target - context.pose.heading() > PI / 2.0 {
                angle_to_target -= PI;
                linear_voltage = -linear_voltage;
            } else if angle_to_target - context.pose.heading() < -PI / 2.0 {
                angle_to_target += PI;
                linear_voltage = -linear_voltage;
            }
            while angle_to_target - context.pose.heading() > PI {
                angle_to_target -= 2.0 * PI;
            }
            while angle_to_target - context.pose.heading() < -PI {
                angle_to_target += 2.0 * PI;
            }
            self.seeking_pid.setpoint(angle_to_target);
            let rotational_voltage = self
                .seeking_pid
                .next_control_output(context.pose.heading())
                .output;
            let velocity = (context.left_velocity + context.right_velocity) / 2.0;
            // If we are within the tolerances, we are settled
            if self.linear_tolerances.check(distance, velocity) {
                self.settled = true;
                return None;
            }
            // If we are not settled, we need to return the voltage
            Some(
                VoltagePair {
                    left: linear_voltage - rotational_voltage,
                    right: linear_voltage + rotational_voltage,
                }
                .max_voltage(self.max_voltage),
            )
        } else {
            // Find the closest point on the path to the current pose
            let current_t = self
                .path
                .closest_point(context.pose, Some(self.last_t), Some(0.1));
            // Find how far along the path we are
            let path_distance = self.path.length_until(current_t);
            // Calculate the distance and velocity to the end of the path
            let error = self.path_total - path_distance;
            let velocity = (context.left_velocity + context.right_velocity) / 2.0;
            // Are we there yet?
            if self.linear_tolerances.check(error, velocity) {
                self.settled = true;
                return None;
            }
            self.last_path_distance = path_distance;
            self.last_t = current_t;

            // Calculate the linear part of the differential drive
            // We simply use a PID controller while plugging in the error along the path
            let mut linear_voltage = self.linear_pid.next_control_output(path_distance).output;

            // Calculate the rotational part of the differential drive
            // With pure pursuit, we find the intersection of the path and a circle
            // with radius equal to the lookahead distance
            // Calculate the angle to the target point
            if let Some(target_t) =
                self.path
                    .point_on_radius(context.pose, self.lookahead, Some(current_t))
            {
                self.target_point = self.path.evaluate(target_t);
                #[cfg(feature = "unsafe_debug_render")]
                {
                    // SAFETY: This is not safe.
                    let mut display = unsafe { vexide::devices::display::Display::new() };
                    let shape = vexide::devices::display::Circle::new(
                        vexide::devices::math::Point2 {
                            x: (self.target_point.x() * 0.066666667 + 120.0) as i16,
                            y: (120.0 - self.target_point.y() * 0.066666667) as i16,
                        },
                        1,
                    );
                    display.fill(&shape, (0, 255, 0));
                }
            } else if self.target_point.distance(context.pose) <= self.lookahead {
                // If we can't find a target point but we're within the lookahead
                // distance, we can just use the end of the path
                self.final_seeking = true;
            } else {
                // We can't find a target point and we've strayed too far from the path
                // Just use the last target point
            }
            // Calculate the angle to the target point
            let mut angle_to_target = context.pose.angle_to(self.target_point);
            // Adjust angle_to_target to be closest to the context.pose.heading()
            while angle_to_target - context.pose.heading() > PI {
                angle_to_target -= 2.0 * PI;
            }
            while angle_to_target - context.pose.heading() < -PI {
                angle_to_target += 2.0 * PI;
            }

            if angle_to_target - context.pose.heading() > PI / 2.0 {
                angle_to_target -= PI;
                linear_voltage = -linear_voltage;
            } else if angle_to_target - context.pose.heading() < -PI / 2.0 {
                angle_to_target += PI;
                linear_voltage = -linear_voltage;
            }

            self.rotational_pid.setpoint(angle_to_target);
            let rotational_voltage = self
                .rotational_pid
                .next_control_output(context.pose.heading())
                .output;

            Some(
                VoltagePair {
                    left: linear_voltage - rotational_voltage,
                    right: linear_voltage + rotational_voltage,
                }
                .max_voltage(self.max_voltage),
            )
        }
    }
}
