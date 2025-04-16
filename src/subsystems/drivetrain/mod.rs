use core::{cell::RefCell, mem};

use alloc::{boxed::Box, rc::Rc};
use vexide::sync::Barrier;
use vexide_motorgroup::MotorGroup;

use crate::utils::traits::HasHeading;

use super::tracking::TrackingSubsystem;

pub mod actions;

#[derive(Debug, Clone, Copy)]
pub struct VoltagePair {
    pub left: f64,
    pub right: f64,
}

impl From<f64> for VoltagePair {
    fn from(voltage: f64) -> Self {
        Self {
            left: voltage,
            right: voltage,
        }
    }
}

impl VoltagePair {
    /// Scales the voltage pair preserving the ratio while staying under the max
    /// voltage
    #[must_use = "does not mutate original value"]
    pub fn max_voltage(self, max_voltage: f64) -> Self {
        if self.left.abs() <= max_voltage && self.right.abs() <= max_voltage {
            self
        } else {
            let ratio = if self.left.abs() >= self.right.abs() {
                max_voltage / self.left.abs()
            } else {
                max_voltage / self.right.abs()
            };
            VoltagePair {
                left: self.left * ratio,
                right: self.right * ratio,
            }
        }
    }
}

pub struct Drivetrain {
    action: Rc<RefCell<Option<Box<dyn actions::Action>>>>,
    action_finish_barrier: Rc<RefCell<Option<vexide::sync::Barrier>>>,
    _task: vexide::task::Task<()>,
}

#[allow(clippy::await_holding_refcell_ref)]
impl Drivetrain {
    pub fn new<HT: HasHeading + 'static>(
        left: Rc<RefCell<MotorGroup>>,
        right: Rc<RefCell<MotorGroup>>,
        heading_sensor: HT,
        wheel_circumference: f64,
        tracking: TrackingSubsystem,
    ) -> Self {
        let action = Rc::new(RefCell::new(None));
        let action_finish_barrier = Rc::new(RefCell::new(None));
        Drivetrain {
            action: action.clone(),
            action_finish_barrier: action_finish_barrier.clone(),
            _task: vexide::task::spawn(async move {
                loop {
                    let mut action_owned = action.borrow_mut();
                    if let Some(ref mut action_ref) = *action_owned {
                        let position = tracking.context();
                        let mut left = left.borrow_mut();
                        let mut right = right.borrow_mut();
                        let context = actions::ActionContext {
                            offset: position.offset,
                            left_offset: left
                                .position()
                                .map(|x| x.as_revolutions() * wheel_circumference)
                                .unwrap_or(0.0),
                            right_offset: right
                                .position()
                                .map(|x| x.as_revolutions() * wheel_circumference)
                                .unwrap_or(0.0),
                            left_velocity: left.velocity().unwrap_or(0.0) * wheel_circumference,
                            right_velocity: right.velocity().unwrap_or(0.0) * wheel_circumference,
                            heading: position.heading,
                        };
                        if let Some(voltage) = action_ref.update(context) {
                            if let Err(e) = left.set_voltage(voltage.left) {
                                log::error!("Failed to set left voltage: {:?}", e);
                            }
                            if let Err(e) = right.set_voltage(voltage.right) {
                                log::error!("Failed to set right voltage: {:?}", e);
                            }
                            drop(action_owned);
                        } else {
                            _ = left.set_voltage(0.0);
                            _ = right.set_voltage(0.0);
                            *action_owned = None;
                            mem::drop(action_owned);
                            mem::drop(left);
                            mem::drop(right);
                            if let Some(barrier) = action_finish_barrier.borrow().as_ref() {
                                barrier.wait().await;
                            }
                        }
                    } else {
                        mem::drop(action_owned);
                    }
                    vexide::time::sleep(core::time::Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_voltage(&mut self, voltage: VoltagePair) {
        let mut action = self.action.borrow_mut();
        *action = Some(Box::new(actions::VoltageAction { voltage }));
    }

    pub async fn boxed_action(&mut self, new_action: Box<dyn actions::Action>) {
        let mut action = self.action.borrow_mut();
        *action = Some(new_action);
        drop(action);

        let barrier = Barrier::new(2);
        *self.action_finish_barrier.borrow_mut() = Some(barrier);

        self.action_finish_barrier
            .borrow()
            .as_ref()
            .unwrap()
            .wait()
            .await;
    }

    pub async fn action(&mut self, action: impl actions::Action + 'static) {
        self.boxed_action(Box::new(action)).await;
    }

    pub fn cancel_action(&mut self) {
        let mut action = self.action.borrow_mut();
        *action = None;
    }
}
