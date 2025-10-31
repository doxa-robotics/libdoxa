use core::{cell::RefCell, future::Future, sync::atomic::AtomicBool};

use alloc::{boxed::Box, rc::Rc};
use vexide_motorgroup::SharedMotors;

use crate::subsystems::tracking::TrackingData;

use super::tracking::TrackingSubsystem;

pub mod actions;
pub mod drivetrain_pair;

pub use drivetrain_pair::DrivetrainPair;

const LOOP_TIME: f64 = 10.0; // ms

#[allow(clippy::type_complexity)]
pub struct DrivetrainActionFuture {
    settled: Rc<AtomicBool>,
    tracking: TrackingSubsystem,
    callback: Option<RefCell<Box<dyn FnMut(TrackingData)>>>,
}

// SAFETY: single-threaded
unsafe impl Send for DrivetrainActionFuture {}
unsafe impl Sync for DrivetrainActionFuture {}

impl Future for DrivetrainActionFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        if self.settled.load(core::sync::atomic::Ordering::Acquire) {
            core::task::Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            if let Some(callback) = &self.callback {
                (callback.borrow_mut())(self.tracking.current());
            }
            core::task::Poll::Pending
        }
    }
}

impl DrivetrainActionFuture {
    pub fn with_callback(mut self, callback: impl FnMut(TrackingData) + 'static) -> Self {
        self.callback = Some(RefCell::new(Box::new(callback)));
        self
    }
}

#[allow(clippy::type_complexity)]
pub struct Drivetrain {
    action: Rc<RefCell<Option<(Box<dyn actions::Action>, Rc<AtomicBool>)>>>,
    max_voltage: Rc<RefCell<f64>>,
    tracking: TrackingSubsystem,
    _task: vexide::task::Task<()>,
}

// SAFETY: vex is single-threaded
unsafe impl Send for Drivetrain {}
unsafe impl Sync for Drivetrain {}

#[allow(clippy::await_holding_refcell_ref)]
impl Drivetrain {
    pub fn new(
        mut left: SharedMotors,
        mut right: SharedMotors,
        max_voltage: f64,
        tracking: TrackingSubsystem,
        max_acceleration: f64, // rpm/s
    ) -> Self {
        let max_acceleration_loop = max_acceleration * LOOP_TIME / 1000.0;
        let action = Rc::new(RefCell::new(None));
        let max_voltage = Rc::new(RefCell::new(max_voltage));
        Drivetrain {
            action: action.clone(),
            max_voltage: max_voltage.clone(),
            tracking: tracking.clone(),
            _task: vexide::task::spawn(async move {
                let last_max_voltage = 0.0;
                let mut last_left_rpm = 0.0;
                let mut last_right_rpm = 0.0;
                loop {
                    {
                        let max_voltage = max_voltage.borrow();
                        if *max_voltage != last_max_voltage {
                            // Update the max voltage
                            _ = left.set_voltage_limit(*max_voltage);
                            _ = right.set_voltage_limit(*max_voltage);
                        }
                    }
                    {
                        let mut action_owned = action.borrow_mut();
                        if let Some(ref mut action_ref) = *action_owned {
                            // Get the tracking position
                            let data = tracking.current();
                            // Assemble the action context
                            let context = actions::ActionContext { data };
                            // Run the action
                            if !action_ref.1.load(core::sync::atomic::Ordering::Acquire)
                                && let Some(mut voltage) = action_ref.0.update(context)
                            {
                                // If the action is still running
                                if tracking.reverse() {
                                    // Rotate the robot in the opposite direction
                                    // if the tracking subsystem is reversed
                                    voltage = voltage.reverse();
                                }
                                // Scale the voltage to be under the max voltage
                                match voltage.units {
                                    drivetrain_pair::DrivetrainUnits::Voltage => {
                                        voltage = voltage.max(*max_voltage.borrow());
                                        // Set the voltage
                                        if let Err(e) = left.set_voltage(voltage.left) {
                                            log::error!("Failed to set left voltage: {:?}", e);
                                        }
                                        if let Err(e) = right.set_voltage(voltage.right) {
                                            log::error!("Failed to set right voltage: {:?}", e);
                                        }
                                    }
                                    drivetrain_pair::DrivetrainUnits::RPM => {
                                        // Set the RPM
                                        if voltage.left > last_left_rpm {
                                            voltage.left = voltage
                                                .left
                                                .min(last_left_rpm + max_acceleration_loop);
                                        } else if voltage.left < last_left_rpm {
                                            voltage.left = voltage
                                                .left
                                                .max(last_left_rpm - max_acceleration_loop);
                                        }
                                        if voltage.right > last_right_rpm {
                                            voltage.right = voltage
                                                .right
                                                .min(last_right_rpm + max_acceleration_loop);
                                        } else if voltage.right < last_right_rpm {
                                            voltage.right = voltage
                                                .right
                                                .max(last_right_rpm - max_acceleration_loop);
                                        }
                                        last_left_rpm = voltage.left;
                                        last_right_rpm = voltage.right;
                                        if let Err(e) = left.set_velocity(voltage.left as i32) {
                                            log::error!("Failed to set left RPM: {:?}", e);
                                        }
                                        if let Err(e) = right.set_velocity(voltage.right as i32) {
                                            log::error!("Failed to set right RPM: {:?}", e);
                                        }
                                    }
                                }
                                drop(action_owned);
                            } else {
                                // Zero out the motors if the action is done
                                _ = left.set_voltage(0.0);
                                _ = right.set_voltage(0.0);
                                // Notify the main task that the action is done
                                action_ref
                                    .1
                                    .store(true, core::sync::atomic::Ordering::SeqCst);
                            }
                        }
                    }
                    vexide::time::sleep(core::time::Duration::from_millis(10)).await;
                }
            }),
        }
    }

    pub fn set_voltage(&mut self, voltage: DrivetrainPair) {
        let mut action = self.action.borrow_mut();
        *action = Some((
            Box::new(actions::VoltageAction { voltage }),
            Rc::new(AtomicBool::new(false)),
        ));
    }

    pub fn set_max_voltage(&mut self, max_voltage: f64) {
        let mut max_voltage_ref = self.max_voltage.borrow_mut();
        *max_voltage_ref = max_voltage;
    }

    pub fn boxed_action(&mut self, new_action: Box<dyn actions::Action>) -> DrivetrainActionFuture {
        let mut action = self.action.borrow_mut();
        let bool = Rc::new(AtomicBool::new(false));
        *action = Some((new_action, bool.clone()));
        drop(action);

        DrivetrainActionFuture {
            callback: None,
            settled: bool,
            tracking: self.tracking.clone(),
        }
    }

    pub fn action(&mut self, action: impl actions::Action + 'static) -> DrivetrainActionFuture {
        self.boxed_action(Box::new(action))
    }

    pub fn cancel_action(&mut self) {
        let mut action = self.action.borrow_mut();
        *action = None;
    }
}
