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

    /// Reverses the voltage pair
    ///
    /// Left becomes right and right becomes left
    #[must_use = "does not mutate original value"]
    pub fn reverse(self) -> Self {
        VoltagePair {
            left: self.right,
            right: self.left,
        }
    }

    /// Returns the average of the left and right voltages
    #[must_use = "does not mutate original value"]
    pub fn average(self) -> f64 {
        (self.left + self.right) / 2.0
    }
}
