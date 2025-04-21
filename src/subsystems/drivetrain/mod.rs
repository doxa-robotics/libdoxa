use core::{cell::RefCell, mem};

use alloc::{boxed::Box, rc::Rc};
use vexide::sync::Barrier;
use vexide_motorgroup::MotorGroup;

use super::tracking::TrackingSubsystem;

pub mod actions;
mod voltage_pair;

pub use voltage_pair::VoltagePair;

pub struct Drivetrain {
    action: Rc<RefCell<Option<Box<dyn actions::Action>>>>,
    action_finish_barrier: Rc<RefCell<Option<vexide::sync::Barrier>>>,
    max_voltage: Rc<RefCell<f64>>,
    _task: vexide::task::Task<()>,
}

#[allow(clippy::await_holding_refcell_ref)]
impl Drivetrain {
    pub fn new(
        left: Rc<RefCell<MotorGroup>>,
        right: Rc<RefCell<MotorGroup>>,
        wheel_circumference: f64,
        max_voltage: f64,
        tracking: Rc<RefCell<TrackingSubsystem>>,
    ) -> Self {
        let action = Rc::new(RefCell::new(None));
        let action_finish_barrier = Rc::new(RefCell::new(None));
        let max_voltage = Rc::new(RefCell::new(max_voltage));
        Drivetrain {
            action: action.clone(),
            action_finish_barrier: action_finish_barrier.clone(),
            max_voltage: max_voltage.clone(),
            _task: vexide::task::spawn(async move {
                loop {
                    let mut action_owned = action.borrow_mut();
                    if let Some(ref mut action_ref) = *action_owned {
                        // Get the tracking position
                        let tracking = tracking.borrow();
                        let position = tracking.pose();
                        let mut left = left.borrow_mut();
                        let mut right = right.borrow_mut();
                        // Assemble the action context
                        let context = actions::ActionContext {
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
                            pose: position,
                        };
                        // Run the action
                        if let Some(mut voltage) = action_ref.update(context) {
                            // If the action is still running
                            if tracking.reverse() {
                                // Rotate the robot in the opposite direction
                                // if the tracking subsystem is reversed
                                voltage = voltage.reverse();
                            }
                            // Scale the voltage to be under the max voltage
                            voltage = voltage.max_voltage(*max_voltage.borrow());
                            // Set the voltage
                            if let Err(e) = left.set_voltage(voltage.left) {
                                log::error!("Failed to set left voltage: {:?}", e);
                            }
                            if let Err(e) = right.set_voltage(voltage.right) {
                                log::error!("Failed to set right voltage: {:?}", e);
                            }
                            drop(action_owned);
                        } else {
                            // Zero out the motors if the action is done
                            _ = left.set_voltage(0.0);
                            _ = right.set_voltage(0.0);
                            *action_owned = None;
                            mem::drop(action_owned);
                            mem::drop(left);
                            mem::drop(right);
                            mem::drop(tracking);
                            // Notify the main task that the action is done
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

    pub fn set_max_voltage(&mut self, max_voltage: f64) {
        let mut max_voltage_ref = self.max_voltage.borrow_mut();
        *max_voltage_ref = max_voltage;
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
