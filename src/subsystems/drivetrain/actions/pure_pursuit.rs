use nalgebra::Point2;
use pid::Pid;
use vexide::math::Angle;

use crate::{
    path_planner::Path, subsystems::drivetrain::DrivetrainPair, utils::settling::Tolerances,
};

use super::{BoomerangAction, config::ActionConfig};

#[derive(Debug)]
pub struct PurePursuitAction<T: Path> {
    // Cached values
    path_total: f64,
    target_point: Point2<f64>,
    lookahead: f64,
    end_point: Point2<f64>,

    // State
    settled: bool,
    last_t: f64,
    final_seeking: Option<BoomerangAction>,

    // PIDs
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,

    // Configuration
    path: T,
    disable_seeking_distance: f64,
    linear_tolerances: Tolerances,
    reverse: bool,
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
            linear_pid: config.linear_pid(0.0),
            path,
            last_t: 0.0,
            settled: false,
            final_seeking: None,
            lookahead: config.pursuit_lookahead,
            rotational_pid: config.pursuit_turn_pid(0.0),
            linear_tolerances: config.linear_tolerances(),
            config,
            reverse: false,
        }
    }

    /// Sets this action to be reversed.
    ///
    /// When reversed, the robot will drive backwards along the path,
    /// attempting to face the *back* of the robot towards the direction of
    /// travel.
    pub fn reversed(mut self) -> Self {
        self.reverse = true;
        self
    }
}

impl<T: Path> super::Action for PurePursuitAction<T> {
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        // If we are settled, we don't need to do anything
        if self.settled {
            return None;
        }
        if let Some(mut action) = self.final_seeking {
            // If we are in final seeking mode, just run that action
            action.update(context)
        } else {
            // Find the closest point on the path to the current pose
            let current_t =
                self.path
                    .closest_point(context.data.offset, Some(self.last_t), Some(0.1));
            // Find how far along the path we are
            let path_distance = self.path.length_until(current_t);
            // Calculate the distance and velocity to the end of the path
            let linear_error = self.path_total - path_distance;
            let linear_velocity = context.data.linear_velocity();
            // Are we there yet?
            if self.linear_tolerances.check(linear_error, linear_velocity) {
                self.settled = true;
                return None;
            }
            self.last_t = current_t;

            // If we're within the disable seeking distance, let's just start seeking
            // the end of the path
            if nalgebra::distance(&self.target_point, &context.data.offset)
                < self.disable_seeking_distance
            {
                self.final_seeking = Some(BoomerangAction::new(
                    self.end_point,
                    Angle::from_radians(self.path.evaluate_angle(1.0)),
                    self.config,
                ));
                return self.final_seeking.as_mut().unwrap().update(context);
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
            } else {
                // We can't find a target point and we've strayed too far from the path
                log::error!(
                    "Pure pursuit: unable to find target point on path! {:?}",
                    context.data.offset
                );
                // Fallback to seeking the previous target point
            }

            // Calculate the closest angular error to the target point
            let angular_error = (Angle::from_radians(
                (self.target_point.y - context.data.offset.y)
                    .atan2(self.target_point.x - context.data.offset.x),
            ) - (context.data.heading
                + if self.reverse {
                    // Reverse the heading by 180 degrees if we're reversed
                    Angle::HALF_TURN
                } else {
                    Angle::ZERO
                }))
            .wrapped_half();

            // Calculate the rotational voltage
            let rotational_voltage = self
                .rotational_pid
                // in principle this should be negative but
                // TODO: verify that the sign is correct
                .next_control_output(-angular_error.as_radians())
                .output;

            // Calculate the linear part of the differential drive
            let linear_voltage = self.linear_pid.next_control_output(-linear_error).output
                // scalar to reduce speed on turns. more info in boomerang action
                * angular_error.cos().max(0.0);

            Some(DrivetrainPair {
                left: linear_voltage - rotational_voltage,
                right: linear_voltage + rotational_voltage,
                units: crate::subsystems::drivetrain::drivetrain_pair::DrivetrainUnits::RPM,
            })
        }
    }
}
