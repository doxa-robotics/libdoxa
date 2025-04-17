use alloc::boxed::Box;
use alloc::vec::Vec;

use crate::path_planner::Path;
use crate::utils::pose::Pose;
use vexide::float::Float as _;

#[derive(Debug)]
/// A struct representing a compound path, which is a collection of paths
/// that are connected together. The paths must be connected at their endpoints.
pub struct CompoundPath {
    paths: Vec<Box<dyn Path>>,
    lengths: Vec<f64>, // Cumulative lengths of the paths
    total_length: f64,
    path_t: f64,
}

impl CompoundPath {
    pub fn new(paths: Vec<Box<dyn Path>>) -> Self {
        assert!(
            !paths.is_empty(),
            "CompoundPath must contain at least one path"
        );

        let mut lengths = Vec::new();
        let mut total_length = 0.0;

        for i in 0..paths.len() {
            if i > 0 {
                let last_pose = paths[i - 1].evaluate(1.0);
                let first_pose = paths[i].evaluate(0.0);
                assert!(
                    last_pose == first_pose,
                    "Endpoints of paths do not match at index {}",
                    i
                );
            }

            total_length += paths[i].length();
            lengths.push(total_length);
        }

        Self {
            path_t: 1.0 / paths.len() as f64,
            paths,
            lengths,
            total_length,
        }
    }
}

impl Path for CompoundPath {
    fn length_until(&self, t: f64) -> f64 {
        // If t is 1.0, we are at the end of the last path
        if t == 1.0 {
            return self.total_length;
        }
        // Find which path t is in
        let path = (self.paths.len() as f64 * t).floor() as isize;
        // If path is less than 0, use the first path. If it's more than the
        // number of paths, use the last path.
        let path = if path < 0 {
            0
        } else if path >= self.paths.len() as isize {
            self.paths.len() - 1
        } else {
            path as usize
        };
        // Find t along that path
        let local_t = (t - path as f64 * self.path_t) / self.path_t;
        self.paths[path].length_until(local_t)
            + if path > 0 {
                self.lengths[path - 1]
            } else {
                0.0
            }
    }

    fn evaluate(&self, t: f64) -> Pose {
        // Find which path t is in
        let path = (self.paths.len() as f64 * t).floor() as isize;
        // If path is less than 0, use the first path. If it's more than the
        // number of paths, use the last path.
        let path = if path < 0 {
            0
        } else if path >= self.paths.len() as isize {
            self.paths.len() - 1
        } else {
            path as usize
        };
        // Find t along that path
        let local_t = (t - path as f64 * self.path_t) / self.path_t;
        // Evaluate the path at the given t
        self.paths[path].evaluate(local_t)
    }
}
