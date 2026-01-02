use core::time::Duration;

use pid::Pid;

use crate::utils::settling::Tolerances;

#[derive(Clone, Debug, Copy)]
pub struct ActionConfig {
    pub linear_kp: f64,
    pub linear_kp_limit: f64,
    pub linear_ki: f64,
    pub linear_ki_limit: f64,
    pub linear_kd: f64,
    pub linear_kd_limit: f64,
    pub linear_limit: f64,

    pub turn_kp: f64,
    pub turn_kp_limit: f64,
    pub turn_ki: f64,
    pub turn_ki_limit: f64,
    pub turn_kd: f64,
    pub turn_kd_limit: f64,
    pub turn_limit: f64,

    pub pursuit_turn_kp: f64,
    pub pursuit_turn_kp_limit: f64,
    pub pursuit_turn_ki: f64,
    pub pursuit_turn_ki_limit: f64,
    pub pursuit_turn_kd: f64,
    pub pursuit_turn_kd_limit: f64,
    pub pursuit_turn_limit: f64,
    pub pursuit_lookahead: f64,

    pub boomerang_lead: f64,
    pub boomerang_close: f64,

    pub linear_error_tolerance: f64,
    pub linear_velocity_tolerance: f64,
    pub linear_tolerance_duration: Duration,
    pub linear_timeout: Duration,

    pub turn_error_tolerance: f64,
    pub turn_velocity_tolerance: f64,
    pub turn_tolerance_duration: Duration,
    pub turn_timeout: Duration,
}

impl ActionConfig {
    pub fn linear_pid(&self, setpoint: f64) -> Pid<f64> {
        let mut pid = Pid::new(setpoint, self.linear_limit);
        pid.p(self.linear_kp, self.linear_kp_limit);
        pid.i(self.linear_ki, self.linear_ki_limit);
        pid.d(self.linear_kd, self.linear_kd_limit);
        pid
    }

    pub fn turn_pid(&self, setpoint: f64) -> Pid<f64> {
        let mut pid = Pid::new(setpoint, self.turn_limit);
        pid.p(self.turn_kp, self.turn_kp_limit);
        pid.i(self.turn_ki, self.turn_ki_limit);
        pid.d(self.turn_kd, self.turn_kd_limit);
        pid
    }

    pub fn pursuit_turn_pid(&self, setpoint: f64) -> Pid<f64> {
        let mut pid = Pid::new(setpoint, self.pursuit_turn_limit);
        pid.p(self.pursuit_turn_kp, self.pursuit_turn_kp_limit);
        pid.i(self.pursuit_turn_ki, self.pursuit_turn_ki_limit);
        pid.d(self.pursuit_turn_kd, self.pursuit_turn_kd_limit);
        pid
    }

    pub fn linear_tolerances(&self) -> Tolerances {
        Tolerances::new()
            .error_tolerance(self.linear_error_tolerance)
            .tolerance_duration(self.linear_tolerance_duration)
            .velocity_tolerance(self.linear_velocity_tolerance)
            .timeout(self.linear_timeout)
    }

    pub fn turn_tolerances(&self) -> Tolerances {
        Tolerances::new()
            .error_tolerance(self.turn_error_tolerance)
            .tolerance_duration(self.turn_tolerance_duration)
            .velocity_tolerance(self.turn_velocity_tolerance)
            .timeout(self.turn_timeout)
    }

    pub fn with_boomerang_lead(mut self, lead: f64) -> Self {
        self.boomerang_lead = lead;
        self
    }

    // #region: Builder
    pub fn with_linear_kp(mut self, linear_kp: f64) -> Self {
        self.linear_kp = linear_kp;
        self
    }
    pub fn with_linear_kp_limit(mut self, linear_kp_limit: f64) -> Self {
        self.linear_kp_limit = linear_kp_limit;
        self
    }
    pub fn with_linear_ki(mut self, linear_ki: f64) -> Self {
        self.linear_ki = linear_ki;
        self
    }
    pub fn with_linear_ki_limit(mut self, linear_ki_limit: f64) -> Self {
        self.linear_ki_limit = linear_ki_limit;
        self
    }
    pub fn with_linear_kd(mut self, linear_kd: f64) -> Self {
        self.linear_kd = linear_kd;
        self
    }
    pub fn with_linear_kd_limit(mut self, linear_kd_limit: f64) -> Self {
        self.linear_kd_limit = linear_kd_limit;
        self
    }
    pub fn with_linear_limit(mut self, linear_limit: f64) -> Self {
        self.linear_limit = linear_limit;
        self
    }
    pub fn with_turn_kp(mut self, turn_kp: f64) -> Self {
        self.turn_kp = turn_kp;
        self
    }
    pub fn with_turn_kp_limit(mut self, turn_kp_limit: f64) -> Self {
        self.turn_kp_limit = turn_kp_limit;
        self
    }
    pub fn with_turn_ki(mut self, turn_ki: f64) -> Self {
        self.turn_ki = turn_ki;
        self
    }
    pub fn with_turn_ki_limit(mut self, turn_ki_limit: f64) -> Self {
        self.turn_ki_limit = turn_ki_limit;
        self
    }
    pub fn with_turn_kd(mut self, turn_kd: f64) -> Self {
        self.turn_kd = turn_kd;
        self
    }
    pub fn with_turn_kd_limit(mut self, turn_kd_limit: f64) -> Self {
        self.turn_kd_limit = turn_kd_limit;
        self
    }
    pub fn with_turn_limit(mut self, turn_limit: f64) -> Self {
        self.turn_limit = turn_limit;
        self
    }
    pub fn with_pursuit_turn_kp(mut self, pursuit_turn_kp: f64) -> Self {
        self.pursuit_turn_kp = pursuit_turn_kp;
        self
    }
    pub fn with_pursuit_turn_kp_limit(mut self, pursuit_turn_kp_limit: f64) -> Self {
        self.pursuit_turn_kp_limit = pursuit_turn_kp_limit;
        self
    }
    pub fn with_pursuit_turn_ki(mut self, pursuit_turn_ki: f64) -> Self {
        self.pursuit_turn_ki = pursuit_turn_ki;
        self
    }
    pub fn with_pursuit_turn_ki_limit(mut self, pursuit_turn_ki_limit: f64) -> Self {
        self.pursuit_turn_ki_limit = pursuit_turn_ki_limit;
        self
    }
    pub fn with_pursuit_turn_kd(mut self, pursuit_turn_kd: f64) -> Self {
        self.pursuit_turn_kd = pursuit_turn_kd;
        self
    }
    pub fn with_pursuit_turn_kd_limit(mut self, pursuit_turn_kd_limit: f64) -> Self {
        self.pursuit_turn_kd_limit = pursuit_turn_kd_limit;
        self
    }
    pub fn with_pursuit_turn_limit(mut self, pursuit_turn_limit: f64) -> Self {
        self.pursuit_turn_limit = pursuit_turn_limit;
        self
    }
    pub fn with_pursuit_lookahead(mut self, pursuit_lookahead: f64) -> Self {
        self.pursuit_lookahead = pursuit_lookahead;
        self
    }
    pub fn with_linear_error_tolerance(mut self, linear_error_tolerance: f64) -> Self {
        self.linear_error_tolerance = linear_error_tolerance;
        self
    }
    pub fn with_linear_velocity_tolerance(mut self, linear_velocity_tolerance: f64) -> Self {
        self.linear_velocity_tolerance = linear_velocity_tolerance;
        self
    }
    pub fn with_linear_tolerance_duration(mut self, linear_tolerance_duration: Duration) -> Self {
        self.linear_tolerance_duration = linear_tolerance_duration;
        self
    }
    pub fn with_linear_timeout(mut self, linear_timeout: Duration) -> Self {
        self.linear_timeout = linear_timeout;
        self
    }
    pub fn with_turn_error_tolerance(mut self, turn_error_tolerance: f64) -> Self {
        self.turn_error_tolerance = turn_error_tolerance;
        self
    }
    pub fn with_turn_velocity_tolerance(mut self, turn_velocity_tolerance: f64) -> Self {
        self.turn_velocity_tolerance = turn_velocity_tolerance;
        self
    }
    pub fn with_turn_tolerance_duration(mut self, turn_tolerance_duration: Duration) -> Self {
        self.turn_tolerance_duration = turn_tolerance_duration;
        self
    }
    pub fn with_turn_timeout(mut self, turn_timeout: Duration) -> Self {
        self.turn_timeout = turn_timeout;
        self
    }
    // #endregion: Builder
}
