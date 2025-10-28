use core::{cell::RefCell, f64};

use alloc::{rc::Rc, vec::Vec};
use nalgebra::{Point2, Rotation2, Vector2};
use vexide::{
    math::Angle,
    prelude::{RotationSensor, SmartDevice},
};

use crate::utils::traits::{HasHeading, HasRotation};

mod tracking_data;
pub mod wheel;
pub use tracking_data::TrackingData;

#[derive(Debug, Clone)]
pub struct TrackingSubsystem {
    current: Rc<RefCell<TrackingData>>,
    reverse: Rc<RefCell<bool>>,
    _task: Rc<vexide::task::Task<()>>,
}

impl TrackingSubsystem {
    /// Creates a new TrackingSubsystem with the given tracking wheels and
    /// heading sensor.
    ///
    /// If there are no tracking wheels in a given direction, the subsystem will
    /// not track movement in that direction. Drivetrain motors should be
    /// passed as a parallel tracking wheel if that is desired.
    ///
    /// The initial pose is at (0, 0) with a heading of 0 radians. It should be
    /// set to the correct initial pose before starting autonomous via
    /// [`set_pose`](Self::set_pose).
    pub fn new<PT: HasRotation + 'static, LT: HasRotation + 'static, HT: HasHeading + 'static>(
        perpendicular_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<PT>>,
        parallel_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<LT>>,
        heading_sensor: HT,
    ) -> Self {
        // Convert the tracking wheels into a solid vector
        let mut perpendicular_tracking_wheels = perpendicular_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<PT>>>();
        let mut parallel_tracking_wheels = parallel_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<LT>>>();
        let current = Rc::new(RefCell::new(TrackingData::default()));
        Self {
            current: current.clone(),
            reverse: Rc::new(RefCell::new(false)),
            _task: Rc::new(vexide::task::spawn(async move {
                // The raw heading is the heading from the heading sensor,
                // before any transformations.
                let mut last_raw_heading = heading_sensor.heading();
                loop {
                    let raw_heading = heading_sensor.heading();
                    let heading_delta = raw_heading - last_raw_heading;
                    last_raw_heading = raw_heading;

                    let last_heading = {
                        let current = current.borrow();
                        current.heading
                        // current is implicitly dropped here
                    };
                    let heading = last_heading + heading_delta;
                    // Average the heading and displacement of the tracking wheels
                    let average_heading = (heading + last_heading) / 2.0;
                    let average_displacement: Vector2<_> =
                        if perpendicular_tracking_wheels.is_empty() {
                            Vector2::zeros()
                        } else {
                            perpendicular_tracking_wheels
                                .iter_mut()
                                .map(|wheel| wheel.local_delta(heading_delta))
                                .sum::<Vector2<_>>()
                                / perpendicular_tracking_wheels.len() as f64
                        } + parallel_tracking_wheels
                            .iter_mut()
                            .map(|wheel| wheel.local_delta(heading_delta))
                            .sum::<Vector2<_>>()
                            / parallel_tracking_wheels.len() as f64;
                    // Update the current pose with the new tracking data.
                    // This is in the original coordinate system.
                    {
                        let mut current = current.borrow_mut();
                        let rotation_matrix =
                            Rotation2::new((average_heading - Angle::HALF_TURN).as_radians());
                        *current = current.advance(
                            current.offset + rotation_matrix * average_displacement,
                            average_heading,
                        );
                    }
                    // TODO: add a way to pass a debug renderer directly to the
                    // tracking subsystem
                    // This is a temporary solution to allow for debugging
                    #[cfg(feature = "unsafe_debug_render")]
                    {
                        // SAFETY: This is not safe.
                        let mut display = unsafe { vexide::display::Display::new() };
                        let shape = vexide::display::Circle::new(
                            vexide::math::Point2 {
                                x: (current.borrow().offset.x * 0.066666667 + 120.0) as i16,
                                y: (120.0 - current.borrow().offset.y * 0.066666667) as i16,
                            },
                            1,
                        );
                        display.fill(&shape, (255, 0, 0));
                    }
                    vexide::time::sleep(RotationSensor::UPDATE_INTERVAL).await;
                }
            })),
        }
    }

    /// The current pose of the robot
    ///
    /// This is the pose of the robot in the transformed coordinate system
    /// used by the `reverse` function. This means that autonomous routes should
    /// be written in the original coordinate system, and then the `reverse`
    /// function can be used to add genericity, if that's a word.
    pub fn current(&self) -> TrackingData {
        let data = *self.current.borrow();
        if *self.reverse.borrow() {
            TrackingData {
                offset: Point2::new(data.offset.x, -data.offset.y),
                heading: Angle::FULL_TURN - data.heading,
                ..data
            }
        } else {
            data
        }
    }

    /// Reset the initial pose of the robot
    ///
    /// Note that this in the transformed coordinate system used by the
    /// `reverse` function.
    pub fn set_current(&mut self, offset: Point2<f64>, heading: Angle) {
        let data = TrackingData {
            offset,
            heading,
            velocity: Vector2::default(),
            angular_velocity: Angle::default(),
            timestamp: Some(std::time::Instant::now()),
            dt: std::time::Duration::default(),
        };
        *self.current.borrow_mut() = if *self.reverse.borrow() {
            TrackingData {
                offset: Point2::new(data.offset.x, -data.offset.y),
                heading: Angle::FULL_TURN - data.heading,
                ..data
            }
        } else {
            data
        };
    }

    /// The reverse state of the tracking subsystem
    ///
    /// This will mirror the pose of the robot over the central line, inverting
    /// the pose heading and y-coordinate.
    pub fn reverse(&self) -> bool {
        *self.reverse.borrow()
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
        *self.reverse.borrow_mut() = reverse;
    }
}
