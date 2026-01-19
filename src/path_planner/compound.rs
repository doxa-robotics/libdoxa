use alloc::boxed::Box;
use alloc::vec::Vec;
use nalgebra::Point2;

use crate::path_planner::Path;

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

    // TODO: move shared logic with evaluate_angle to a helper function

    fn evaluate(&self, t: f64) -> Point2<f64> {
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

    fn evaluate_angle(&self, t: f64) -> f64 {
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
        self.paths[path].evaluate_angle(local_t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    struct MockPath {
        length: f64,
        start: Point2<f64>,
        end: Point2<f64>,
    }

    impl Path for MockPath {
        fn length(&self) -> f64 {
            self.length
        }

        fn length_until(&self, t: f64) -> f64 {
            self.length * t
        }

        fn evaluate(&self, t: f64) -> Point2<f64> {
            Point2::new(
                self.start.x + (self.end.x - self.start.x) * t,
                self.start.y + (self.end.y - self.start.y) * t,
            )
        }

        fn evaluate_angle(&self, _t: f64) -> f64 {
            0.0
        }
    }

    #[test]
    fn test_single_path() {
        let path = Box::new(MockPath {
            length: 10.0,
            start: Point2::new(0.0, 0.0),
            end: Point2::new(10.0, 0.0),
        });
        let compound = CompoundPath::new(vec![path]);
        assert_eq!(compound.length(), 10.0);
    }

    #[test]
    fn test_multiple_paths() {
        let paths = vec![
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(0.0, 0.0),
                end: Point2::new(10.0, 0.0),
            }) as Box<dyn Path>,
            Box::new(MockPath {
                length: 5.0,
                start: Point2::new(10.0, 0.0),
                end: Point2::new(10.0, 5.0),
            }),
        ];
        let compound = CompoundPath::new(paths);
        assert_eq!(compound.length(), 15.0);
    }

    #[test]
    fn test_evaluate_first_half() {
        let paths = vec![
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(0.0, 0.0),
                end: Point2::new(10.0, 0.0),
            }) as Box<dyn Path>,
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(10.0, 0.0),
                end: Point2::new(20.0, 0.0),
            }),
        ];
        let compound = CompoundPath::new(paths);
        let point = compound.evaluate(0.25);
        assert_eq!(point.x, 5.0);
    }

    #[test]
    fn test_evaluate_second_half() {
        let paths = vec![
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(0.0, 0.0),
                end: Point2::new(10.0, 0.0),
            }) as Box<dyn Path>,
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(10.0, 0.0),
                end: Point2::new(20.0, 0.0),
            }),
        ];
        let compound = CompoundPath::new(paths);
        let point = compound.evaluate(0.75);
        assert_eq!(point.x, 15.0);
    }

    #[test]
    #[should_panic]
    fn test_disconnected_paths_panic() {
        let paths = vec![
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(0.0, 0.0),
                end: Point2::new(10.0, 0.0),
            }) as Box<dyn Path>,
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(20.0, 0.0),
                end: Point2::new(30.0, 0.0),
            }),
        ];
        CompoundPath::new(paths);
    }

    #[test]
    fn test_length_until() {
        let paths = vec![
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(0.0, 0.0),
                end: Point2::new(10.0, 0.0),
            }) as Box<dyn Path>,
            Box::new(MockPath {
                length: 10.0,
                start: Point2::new(10.0, 0.0),
                end: Point2::new(20.0, 0.0),
            }),
        ];
        let compound = CompoundPath::new(paths);
        assert_eq!(compound.length_until(0.5), 10.0);
        assert_eq!(compound.length_until(1.0), 20.0);
    }
}
