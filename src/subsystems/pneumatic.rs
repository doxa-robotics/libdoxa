use vexide::devices::adi::digital::LogicLevel;

#[derive(Debug, PartialEq, Eq)]
pub struct PneumaticSubsystem<const N: usize, const LOW_IS_EXTENDED: bool = false> {
    solenoids: [vexide::devices::adi::AdiDigitalOut; N],
}

impl<const N: usize, const LOW_IS_EXTENDED: bool> PneumaticSubsystem<N, LOW_IS_EXTENDED> {
    /// Creates a new `PneumaticSubsystem` with the given solenoids.
    pub fn new(solenoids: [vexide::devices::adi::AdiDigitalOut; N]) -> Self {
        const {
            if N == 0 {
                panic!("Cannot create PneumaticSubsystem with zero solenoids.");
            }
        }
        Self { solenoids }
    }

    /// Extends the piston(s).
    pub fn extend(&mut self) {
        for solenoid in self.solenoids.iter_mut() {
            _ = solenoid.set_level(match LOW_IS_EXTENDED {
                true => LogicLevel::Low,
                false => LogicLevel::High,
            });
        }
    }

    /// Retracts the piston(s).
    pub fn retract(&mut self) {
        for solenoid in self.solenoids.iter_mut() {
            _ = solenoid.set_level(match LOW_IS_EXTENDED {
                true => LogicLevel::High,
                false => LogicLevel::Low,
            });
        }
    }

    /// Toggles the piston(s).
    pub fn toggle(&mut self) {
        for solenoid in self.solenoids.iter_mut() {
            _ = solenoid.toggle();
        }
    }

    /// Returns whether the piston(s) is/are extended.
    pub fn extended(&self) -> bool {
        self.solenoids[0].level().is_ok_and(|level| {
            level
                == match LOW_IS_EXTENDED {
                    true => LogicLevel::Low,
                    false => LogicLevel::High,
                }
        })
    }

    /// Returns whether the piston(s) is/are retracted.
    pub fn retracted(&self) -> bool {
        self.solenoids[0].level().is_ok_and(|level| {
            level
                == match LOW_IS_EXTENDED {
                    true => LogicLevel::High,
                    false => LogicLevel::Low,
                }
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MirroredState {
    /// Like normal. Right is right, left is left.
    Normal,
    /// Mirrored. Right is left, left is right.
    Mirrored,
}

#[derive(Debug, PartialEq, Eq)]
pub struct MirroredPneumaticSubsystem<const N: usize, const LOW_IS_EXTENDED: bool = false> {
    left: PneumaticSubsystem<N, LOW_IS_EXTENDED>,
    right: PneumaticSubsystem<N, LOW_IS_EXTENDED>,
    mirrored_state: MirroredState,
}

impl<const N: usize, const LOW_IS_EXTENDED: bool> MirroredPneumaticSubsystem<N, LOW_IS_EXTENDED> {
    /// Creates a new `MirroredPneumaticSubsystem` with the given solenoids.
    pub fn new(
        left_solenoids: [vexide::devices::adi::AdiDigitalOut; N],
        right_solenoids: [vexide::devices::adi::AdiDigitalOut; N],
        mirrored_state: MirroredState,
    ) -> Self {
        Self {
            left: PneumaticSubsystem::new(left_solenoids),
            right: PneumaticSubsystem::new(right_solenoids),
            mirrored_state,
        }
    }

    /// Returns the dominant side of the subsystem (i.e., normally right, mirrored left).
    pub fn dominant(&mut self) -> &mut PneumaticSubsystem<N, LOW_IS_EXTENDED> {
        match self.mirrored_state {
            MirroredState::Normal => &mut self.right,
            MirroredState::Mirrored => &mut self.left,
        }
    }

    /// Returns the non-dominant side of the subsystem (i.e., normally left, mirrored right).
    pub fn non_dominant(&mut self) -> &mut PneumaticSubsystem<N, LOW_IS_EXTENDED> {
        match self.mirrored_state {
            MirroredState::Normal => &mut self.left,
            MirroredState::Mirrored => &mut self.right,
        }
    }

    /// Sets the mirrored state of the subsystem
    pub fn set_mirrored_state(&mut self, mirrored_state: MirroredState) {
        self.mirrored_state = mirrored_state;
    }

    /// Gets the current mirrored state of the subsystem
    pub fn mirrored_state(&self) -> MirroredState {
        self.mirrored_state
    }
}
