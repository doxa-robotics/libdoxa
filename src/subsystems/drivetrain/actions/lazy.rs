use core::fmt::Debug;

use alloc::boxed::Box;

use crate::{subsystems::drivetrain::DrivetrainPair, utils::pose::Pose};

use super::Action;

pub struct LazyAction<T: Action> {
    action: Option<T>,
    initializer: Option<Box<dyn FnOnce(Pose) -> T + 'static>>,
}

impl<T: Action> Debug for LazyAction<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LazyAction")
            .field("action", &self.action)
            .finish()
    }
}

impl<T: Action> LazyAction<T> {
    pub fn new(initializer: impl FnOnce(Pose) -> T + 'static) -> Self {
        Self {
            action: None,
            initializer: Some(Box::new(initializer)),
        }
    }
}

impl<T: Action> super::Action for LazyAction<T> {
    fn update(&mut self, context: super::ActionContext) -> Option<DrivetrainPair> {
        if self.action.is_none() {
            if let Some(initializer) = self.initializer.take() {
                self.action = Some(initializer(context.pose));
            }
        }

        if let Some(action) = &mut self.action {
            action.update(context)
        } else {
            None
        }
    }
}
