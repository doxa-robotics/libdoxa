#[derive(Debug, Clone, Copy)]
pub enum DrivetrainUnits {
    /// Volts
    Voltage,
    /// Velocity built-in PID: RPM
    RPM,
}

#[derive(Debug, Clone, Copy)]
pub struct DrivetrainPair {
    pub left: f64,
    pub right: f64,
    pub units: DrivetrainUnits,
}

impl From<f64> for DrivetrainPair {
    fn from(voltage: f64) -> Self {
        Self {
            left: voltage,
            right: voltage,
            units: DrivetrainUnits::Voltage,
        }
    }
}

impl DrivetrainPair {
    /// Constructs a new DrivetrainPair using RPM units
    pub fn new_rpm(left: f64, right: f64) -> Self {
        DrivetrainPair {
            left,
            right,
            units: DrivetrainUnits::RPM,
        }
    }

    /// Scales the voltage pair preserving the ratio while staying under the max
    /// voltage
    #[must_use = "does not mutate original value"]
    pub fn max(self, max: f64) -> Self {
        if self.left.abs() <= max && self.right.abs() <= max {
            self
        } else {
            let ratio = if self.left.abs() >= self.right.abs() {
                max / self.left.abs()
            } else {
                max / self.right.abs()
            };
            DrivetrainPair {
                left: self.left * ratio,
                right: self.right * ratio,
                units: self.units,
            }
        }
    }

    /// Reverses the voltage pair
    ///
    /// Left becomes right and right becomes left
    #[must_use = "does not mutate original value"]
    pub fn reverse(self) -> Self {
        DrivetrainPair {
            left: self.right,
            right: self.left,
            units: self.units,
        }
    }

    /// Returns the average of the left and right values
    #[must_use = "does not mutate original value"]
    pub fn average(self) -> f64 {
        (self.left + self.right) / 2.0
    }
}
