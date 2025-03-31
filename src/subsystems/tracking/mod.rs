use core::cell::RefCell;

use alloc::{rc::Rc, vec::Vec};
use vexide::prelude::{RotationSensor, SmartDevice};

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
                let mut last_heading = heading_sensor.heading();
                loop {
                    let heading = heading_sensor.heading();
                    let heading_delta = heading - last_heading;
                    last_heading = heading;
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
                    // SAFETY: This is not safe.
                    let mut display = unsafe { vexide::devices::display::Display::new() };
                    let shape = vexide::devices::display::Circle::new(
                        vexide::devices::math::Point2 {
                            x: (offset.borrow().x / 10.0 + 100.0) as i16,
                            y: (offset.borrow().y / 10.0 + 100.0) as i16,
                        },
                        1,
                    );
                    display.fill(&shape, (255, 0, 0));
                    vexide::io::println!("{:.2?},{:.2?}", offset.borrow().x, offset.borrow().y);
                    vexide::time::sleep(RotationSensor::UPDATE_INTERVAL).await;
                }
            }),
        }
    }

    pub fn position(&self) -> Vec2<f64> {
        *self.offset.borrow()
    }
}
