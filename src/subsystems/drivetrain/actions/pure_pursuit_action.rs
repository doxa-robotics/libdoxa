use pid::Pid;

use crate::{
    path_planner::Path,
    subsystems::drivetrain::VoltagePair,
    utils::{pose::Pose, settling::Tolerances},
};

#[derive(Debug)]
pub struct PurePursuitAction<T: Path> {
    path: T,
    path_total: f32,
    rotational_pid: Pid<f64>,
    linear_pid: Pid<f64>,
    linear_tolerances: Tolerances,
    last_t: f32,
    target_point: Pose,
    lookahead: f32,
}

impl<T: Path> PurePursuitAction<T> {
    pub fn new(
        path: T,
        rotational_pid: Pid<f64>,
        mut linear_pid: Pid<f64>,
        linear_tolerances: Tolerances,
        lookahead: f32,
    ) -> Self {
        linear_pid.setpoint(0.0);
        Self {
            path_total: path.length(),
            path,
            rotational_pid,
            linear_pid,
            linear_tolerances,
            last_t: 0.0,
            target_point: Pose::default(),
            lookahead,
        }
    }
}

impl<T: Path> super::Action for PurePursuitAction<T> {
    fn update(&mut self, context: super::ActionContext) -> Option<VoltagePair> {
        let current_pose = Pose::new(
            context.offset.x as f32,
            context.offset.y as f32,
            context.heading as f32,
        );
        let current_t = self
            .path
            .closest_point(current_pose, Some(self.last_t), Some(0.1));
        self.last_t = current_t;
        let path_distance = self.path.length_until(current_t);
        let error = self.path_total - path_distance;
        let linear_voltage = self.linear_pid.next_control_output(error as f64).output;
        if let Some(target_t) =
            self.path
                .point_on_radius(current_pose, self.lookahead, Some(current_t))
        {
            self.target_point = self.path.evaluate(target_t);
        }
        todo!("finish implementing")
    }
}
