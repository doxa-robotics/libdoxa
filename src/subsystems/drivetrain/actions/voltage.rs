use crate::subsystems::drivetrain::DrivetrainPair;

#[derive(Debug)]
pub struct VoltageAction {
    pub voltage: DrivetrainPair,
}

impl super::Action for VoltageAction {
    fn update(&mut self, _context: super::ActionContext) -> Option<DrivetrainPair> {
        Some(self.voltage)
    }
}
