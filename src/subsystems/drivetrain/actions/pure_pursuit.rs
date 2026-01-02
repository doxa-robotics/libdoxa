use core::f64::consts::PI;

use nalgebra::Point2;
use pid::Pid;
use vexide::math::Angle;

use crate::{
    path_planner::Path, subsystems::drivetrain::DrivetrainPair, utils::settling::Tolerances,
};

use super::{BoomerangAction, boomerang::turning_linear_scalar_curve, config::ActionConfig};

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
    target_point: Point2<f64>,
    lookahead: f64,
    settled: bool,
    end_point: Point2<f64>,
    final_seeking: Option<BoomerangAction>,
    config: ActionConfig,
}

impl<T: Path> PurePursuitAction<T> {
    pub fn new(path: T, disable_seeking_distance: Option<f64>, config: ActionConfig) -> Self {
        let path_total = path.length();
        Self {
            end_point: path.evaluate(1.0),
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
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        // If we are settled, we don't need to do anything
        if self.settled {
            return None;
        }
        if let Some(mut action) = self.final_seeking {
            action.update(context)
        } else {
            // Find the closest point on the path to the current pose
            let current_t =
                self.path
                    .closest_point(context.data.offset, Some(self.last_t), Some(0.1));
            // Find how far along the path we are
            let path_distance = self.path.length_until(current_t);
            // Calculate the distance and velocity to the end of the path
            let error = self.path_total - path_distance;
            let velocity = context.data.linear_velocity();
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
            if nalgebra::distance(&self.target_point, &context.data.offset)
                < self.disable_seeking_distance
            {
                self.final_seeking = Some(BoomerangAction::new(self.end_point, self.config));
                return None;
            }

            // Calculate the rotational part of the differential drive
            // With pure pursuit, we find the intersection of the path and a circle
            // with radius equal to the lookahead distance
            // Calculate the angle to the target point
            if let Some(target_t) =
                self.path
                    .point_on_radius(context.data.offset, self.lookahead, Some(current_t))
            {
                self.target_point = self.path.evaluate(target_t);
                #[cfg(feature = "unsafe_debug_render")]
                if matches!(
                    // Only render if we're doing autonomous or if disabled
                    // with a legacy competition switch (what we use for
                    // testing)
                    vexide::competition::mode(),
                    vexide::competition::CompetitionMode::Autonomous
                        | vexide::competition::CompetitionMode::Disabled
                ) && matches!(
                    vexide::competition::system(),
                    Some(vexide::competition::CompetitionSystem::CompetitionSwitch)
                ) {
                    // SAFETY: This is not safe.
                    let mut display = unsafe { vexide::display::Display::new() };
                    let shape = vexide::display::Circle::new(
                        vexide::math::Point2 {
                            x: (self.target_point.x * 0.066666667 + 120.0) as i16,
                            y: (120.0 - self.target_point.y * 0.066666667) as i16,
                        },
                        1,
                    );
                    display.fill(&shape, (0, 255, 0));
                }
            } else if nalgebra::distance(&self.target_point, &context.data.offset) <= self.lookahead
            {
                // If we can't find a target point but we're within the lookahead
                // distance, we can just use the end of the path
                self.final_seeking = Some(BoomerangAction::new(self.end_point, self.config));
            } else {
                // We can't find a target point and we've strayed too far from the path
                self.final_seeking = Some(BoomerangAction::new(self.end_point, self.config));
            }
            // Calculate the angle to the target point
            let mut angle_to_target = (self.target_point.y - context.data.offset.y)
                .atan2(self.target_point.x - context.data.offset.x);
            // Adjust angle_to_target to be closest to the context.pose.heading()
            while angle_to_target - context.data.heading.as_radians() > PI {
                angle_to_target -= 2.0 * PI;
            }
            while angle_to_target - context.data.heading.as_radians() < -PI {
                angle_to_target += 2.0 * PI;
            }

            if angle_to_target - context.data.heading.as_radians() > PI / 2.0 {
                angle_to_target -= PI;
                linear_voltage = -linear_voltage;
            } else if angle_to_target - context.data.heading.as_radians() < -PI / 2.0 {
                angle_to_target += PI;
                linear_voltage = -linear_voltage;
            }

            self.rotational_pid.setpoint(angle_to_target);
            let rotational_voltage = self
                .rotational_pid
                .next_control_output(context.data.heading.as_radians())
                .output;

            // First, scale the linear voltage.
            let linear_scalar = turning_linear_scalar_curve(Angle::from_radians(
                context.data.heading.as_radians() - angle_to_target,
            ));
            linear_voltage *= linear_scalar;
            Some(DrivetrainPair {
                left: linear_voltage - rotational_voltage,
                right: linear_voltage + rotational_voltage,
                units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::RPM,
            })
        }
    }
}
