use crate::subsystems::drivetrain::VoltagePair;

#[derive(Debug)]
pub struct VoltageAction {
    pub voltage: VoltagePair,
}

impl super::Action for VoltageAction {
    fn update(&mut self, _context: super::ActionContext) -> Option<VoltagePair> {
        Some(self.voltage)
    }
}
