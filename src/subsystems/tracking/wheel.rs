use core::{error::Error, f64, fmt::Debug};

use snafu::Snafu;
use vexide::{float::Float, prelude::Position};

use crate::utils::{traits::HasRotation, vec2::Vec2};

#[derive(Debug, Snafu)]
pub enum TrackingWheelError<T: Debug + Error + 'static> {
    #[snafu(display("Sensor error: {}", source))]
    Sensor { source: T },
}

#[derive(Debug, Clone, Copy)]
pub enum TrackingWheelMountingDirection {
    /// The tracking wheel is mounted parallel to the robot's forward direction.
    Parallel,
    /// The tracking wheel is mounted perpendicular to the robot's forward direction.
    Perpendicular,
}

pub struct TrackingWheel<T: HasRotation> {
    circumference: f64,
    mounting_offset: f64,
    sensor: T,
    last_position: Position,
    mounting_direction: TrackingWheelMountingDirection,
}

impl<T: HasRotation> TrackingWheel<T> {
    pub fn new(
        circumference: f64,
        mounting_offset: f64,
        mounting_direction: TrackingWheelMountingDirection,
        sensor: T,
    ) -> TrackingWheel<T> {
        Self {
            circumference,
            mounting_offset,
            mounting_direction,
            last_position: sensor.position(),
            sensor,
        }
    }

    pub fn new_parallel(circumference: f64, mounting_offset: f64, sensor: T) -> TrackingWheel<T> {
        Self::new(
            circumference,
            mounting_offset,
            TrackingWheelMountingDirection::Parallel,
            sensor,
        )
    }

    pub fn new_perpendicular(
        circumference: f64,
        mounting_offset: f64,
        sensor: T,
    ) -> TrackingWheel<T> {
        Self::new(
            circumference,
            mounting_offset,
            TrackingWheelMountingDirection::Perpendicular,
            sensor,
        )
    }

    /// Returns the difference between the last reported position and the current position.
    pub fn delta(&mut self) -> f64 {
        let position = self.sensor.position();
        let delta = position - self.last_position;
        self.last_position = position;
        delta.as_revolutions() * self.circumference
    }

    pub fn mounting_offset(&self) -> f64 {
        self.mounting_offset
    }

    pub fn mounting_direction(&self) -> TrackingWheelMountingDirection {
        self.mounting_direction
    }

    /// Returns the local delta of the tracking wheel.
    ///
    /// The frame of reference for the local delta is the y axis facing
    /// in the direction of forward travel.
    pub fn local_delta(&mut self, heading_delta: f64) -> Vec2<f64> {
        match self.mounting_direction {
            TrackingWheelMountingDirection::Parallel => {
                if heading_delta == 0.0 {
                    Vec2::new(0.0, self.delta())
                } else {
                    Vec2::new(
                        0.0,
                        2.0 * (heading_delta / 2.0).sin()
                            * (self.delta() / heading_delta + self.mounting_offset()),
                    )
                }
            }
            TrackingWheelMountingDirection::Perpendicular => {
                if heading_delta == 0.0 {
                    Vec2::new(self.delta(), 0.0)
                } else {
                    Vec2::new(
                        2.0 * (heading_delta / 2.0).sin()
                            * (self.delta() / heading_delta + self.mounting_offset()),
                        0.0,
                    )
                }
            }
        }
    }
}
