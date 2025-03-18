use core::cell::RefCell;

use alloc::{rc::Rc, vec::Vec};
use vexide::{
    float::Float,
    prelude::{RotationSensor, SmartDevice},
};

use crate::utils::{
    traits::{HasHeading, HasRotation},
    vec2::Vec2,
};

pub mod wheel;

#[derive(Debug)]
pub struct TrackingSubsystem {
    offset: Rc<RefCell<Vec2<f64>>>,
    _task: vexide::task::Task<()>,
}

impl TrackingSubsystem {
    pub fn new<PT: HasRotation + 'static, LT: HasRotation + 'static, HT: HasHeading + 'static>(
        perpendicular_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<PT>>,
        parallel_tracking_wheels: impl IntoIterator<Item = wheel::TrackingWheel<LT>>,
        heading_sensor: HT,
        initial_offset: Vec2<f64>,
    ) -> Self {
        let mut perpendicular_tracking_wheels = perpendicular_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<PT>>>();
        let mut parallel_tracking_wheels = parallel_tracking_wheels
            .into_iter()
            .collect::<Vec<wheel::TrackingWheel<LT>>>();
        let offset = Rc::new(RefCell::new(initial_offset));
        Self {
            offset: offset.clone(),
            _task: vexide::task::spawn(async move {
                let last_heading = heading_sensor.heading();
                loop {
                    let heading = heading_sensor.heading();
                    let heading_delta = (heading - last_heading).rem_euclid(360.0);
                    let average_heading = (heading + last_heading) / 2.0;
                    let average_displacement: Vec2<_> = (perpendicular_tracking_wheels
                        .iter_mut()
                        .map(|wheel| wheel.local_delta(heading_delta))
                        .sum::<Vec2<_>>()
                        / perpendicular_tracking_wheels.len() as f64
                        + parallel_tracking_wheels
                            .iter_mut()
                            .map(|wheel| wheel.local_delta(heading_delta))
                            .sum::<Vec2<_>>())
                        / parallel_tracking_wheels.len() as f64;
                    *offset.borrow_mut() += average_displacement.rotated(average_heading);
                    vexide::time::sleep(RotationSensor::UPDATE_INTERVAL).await;
                }
            }),
        }
    }

    pub fn position(&self) -> Vec2<f64> {
        *self.offset.borrow()
    }
}
