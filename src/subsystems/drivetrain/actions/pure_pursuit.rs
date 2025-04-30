use core::f64::consts::PI;

use pid::Pid;

use crate::{
    path_planner::Path,
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

use super::{boomerang::turning_linear_scalar_curve, config::ActionConfig, BoomerangAction};

#[derive(Debug)]
pub struct PurePursuitAction<T: Path> {
    path: T,
    disable_seeking_distance: f64,
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
    final_seeking: Option<BoomerangAction>,
    config: ActionConfig,
}

impl<T: Path> PurePursuitAction<T> {
    pub fn new(path: T, disable_seeking_distance: Option<f64>, config: ActionConfig) -> Self {
        let path_total = path.length();
        Self {
            end_pose: path.evaluate(1.0),
            path_total,
            disable_seeking_distance: disable_seeking_distance.unwrap_or(0.0),
            target_point: path.evaluate(0.0),
            linear_pid: config.linear_pid(path.length()),
            path,
            last_t: 0.0,
            last_path_distance: 0.0,
            settled: false,
            final_seeking: None,
            lookahead: config.pursuit_lookahead,
            rotational_pid: config.pursuit_turn_pid(0.0),
            linear_tolerances: config.linear_tolerances(),
            config,
        }
    }
}

impl<T: Path> super::Action for PurePursuitAction<T> {
    fn update(&mut self, context: super::ActionContext) -> Option<VoltagePair> {
        // If we are settled, we don't need to do anything
        if self.settled {
            return None;
        }
        if let Some(mut action) = self.final_seeking {
            action.update(context)
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

            // If we're within the disable seeking distance, let's just start seeking
            // the end of the path
            if self.target_point.distance(context.pose) < self.disable_seeking_distance {
                self.final_seeking = Some(BoomerangAction::new(self.end_pose, self.config));
                return None;
            }

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
                self.final_seeking = Some(BoomerangAction::new(self.end_pose, self.config));
            } else {
                // We can't find a target point and we've strayed too far from the path
                self.final_seeking = Some(BoomerangAction::new(self.end_pose, self.config));
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

            // First, scale the linear voltage.
            let linear_scalar =
                turning_linear_scalar_curve(context.pose.heading() - angle_to_target);
            linear_voltage *= linear_scalar;
            Some(VoltagePair {
                left: linear_voltage - rotational_voltage,
                right: linear_voltage + rotational_voltage,
            })
        }
    }
}
