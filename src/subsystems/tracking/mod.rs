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
                let mut last_heading = initial.heading() - heading_sensor.heading();
                loop {
                    let heading = initial.heading() - heading_sensor.heading();
                    let heading_delta = heading - last_heading;
                    last_heading = heading;
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
                    {
                        let mut current_pose = current_pose.borrow_mut();
                        let rotation_matrix = Rotation2::new(average_heading - PI / 2.0);
                        current_pose.offset += rotation_matrix * average_displacement;
                        current_pose.heading = average_heading;
                    }
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
                        vexide::time::sleep(RotationSensor::UPDATE_INTERVAL).await;
                    }
                }
            }),
        }
    }

    pub fn pose(&self) -> Pose {
        let pose = *self.pose.borrow();
        if self.reverse {
            Pose::new(pose.offset.x, -pose.offset.y, PI - pose.heading)
        } else {
            pose
        }
    }

    pub fn set_pose(&mut self, pose: Pose) {
        *self.pose.borrow_mut() = pose;
    }

    pub fn reverse(&self) -> bool {
        self.reverse
    }

    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }
}
