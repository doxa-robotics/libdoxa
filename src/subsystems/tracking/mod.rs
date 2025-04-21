use core::{
    cell::RefCell,
    f64::{self, consts::PI},
};

use alloc::{rc::Rc, vec::Vec};
use nalgebra::{Rotation2, Vector2};
use vexide::prelude::{RotationSensor, SmartDevice};

use crate::utils::{
    pose::Pose,
    traits::{HasHeading, HasRotation},
};

pub mod wheel;

#[derive(Debug)]
pub struct TrackingSubsystem {
    pose: Rc<RefCell<Pose>>,
    reverse: bool,
    _task: vexide::task::Task<()>,
}

impl TrackingSubsystem {
    pub fn new<PT: HasRotation + 'static, LT: HasRotation + 'static, HT: HasHeading + 'static>(
        perpendicular_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<PT>>,
        parallel_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<LT>>,
        heading_sensor: HT,
        initial: Pose,
    ) -> Self {
        // Convert the tracking wheels into a solid vector
        let mut perpendicular_tracking_wheels = perpendicular_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<PT>>>();
        let mut parallel_tracking_wheels = parallel_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<LT>>>();
        let current_pose = Rc::new(RefCell::new(initial));
        Self {
            pose: current_pose.clone(),
            reverse: false,
            _task: vexide::task::spawn(async move {
                // Get the current heading to initialize last_heading. Note that
                // the initial heading is taken into account.
                let mut last_heading = initial.heading() - heading_sensor.heading();
                loop {
                    // Get the current heading and calculate the delta while
                    // taking the initial heading into account.
                    let heading = initial.heading() - heading_sensor.heading();
                    let heading_delta = heading - last_heading;
                    last_heading = heading;
                    // Average the heading and displacement of the tracking wheels
                    let average_heading = (heading + last_heading) / 2.0;
                    let average_displacement: Vector2<_> = (perpendicular_tracking_wheels
                        .iter_mut()
                        .map(|wheel| wheel.local_delta(heading_delta))
                        .sum::<Vector2<_>>()
                        / perpendicular_tracking_wheels.len() as f64
                        + parallel_tracking_wheels
                            .iter_mut()
                            .map(|wheel| wheel.local_delta(heading_delta))
                            .sum::<Vector2<_>>())
                        / parallel_tracking_wheels.len() as f64;
                    // Update the current pose with the new tracking data.
                    // This is in the original coordinate system.
                    {
                        let mut current_pose = current_pose.borrow_mut();
                        let rotation_matrix = Rotation2::new(average_heading - PI / 2.0);
                        current_pose.offset += rotation_matrix * average_displacement;
                        current_pose.heading = average_heading;
                    }
                    // TODO: add a way to pass a debug renderer directly to the
                    // tracking subsystem
                    // This is a temporary solution to allow for debugging
                    #[cfg(feature = "unsafe_debug_render")]
                    {
                        // SAFETY: This is not safe.
                        let mut display = unsafe { vexide::devices::display::Display::new() };
                        let shape = vexide::devices::display::Circle::new(
                            vexide::devices::math::Point2 {
                                x: (current_pose.borrow().offset.x * 0.066666667 + 120.0) as i16,
                                y: (120.0 - current_pose.borrow().offset.y * 0.066666667) as i16,
                            },
                            1,
                        );
                        display.fill(&shape, (255, 0, 0));
                    }
                    vexide::time::sleep(RotationSensor::UPDATE_INTERVAL).await;
                }
            }),
        }
    }

    /// The current pose of the robot
    ///
    /// This is the pose of the robot in the transformed coordinate system
    /// used by the `reverse` function. This means that autonomous routes should
    /// be written in the original coordinate system, and then the `reverse`
    /// function can be used to add genericity, if that's a word.
    pub fn pose(&self) -> Pose {
        let pose = *self.pose.borrow();
        if self.reverse {
            Pose::new(pose.offset.x, -pose.offset.y, PI - pose.heading)
        } else {
            pose
        }
    }

    /// Reset the current pose of the robot
    ///
    /// Note that this in the transformed coordinate system used by the `reverse`
    /// function.
    pub fn set_pose(&mut self, pose: Pose) {
        *self.pose.borrow_mut() = {
            if self.reverse {
                Pose::new(pose.offset.x, -pose.offset.y, PI - pose.heading)
            } else {
                pose
            }
        };
    }

    /// The reverse state of the tracking subsystem
    ///
    /// This will mirror the pose of the robot over the central line, inverting
    /// the pose heading and y-coordinate.
    pub fn reverse(&self) -> bool {
        self.reverse
    }

    /// Sets the reverse state of the tracking subsystem
    ///
    /// This will reverse the pose of the robot, so that the robot will
    /// move in the opposite direction as is useful in games where the field
    /// is mirrored across the central line.
    ///
    /// Note that during driver control, this will also cause `Drivetrain`
    /// to reverse the direction of the robot. You probably don't want that, so
    /// you should set the reverse state to be `false` at the beginning of
    /// driver control.
    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }
}
