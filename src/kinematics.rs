use na::{UnitQuaternion, Vector3};
use serde::Deserialize;
use serde_json;

use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

#[derive(Deserialize)]
struct RobotConfiguration {
    frame_anchors: Vec<[f32; 3]>,
    stylus_anchors: Vec<[f32; 3]>,
    stylus_dims: [f32; 3],
}

#[derive(Copy, Clone, PartialEq)]
pub struct Pose {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

impl Pose {
    pub fn new() -> Pose {
        Pose {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.0),
        }
    }

    pub fn transform_vector(&self, v: &Vector3<f32>) -> Vector3<f32> {
        self.position + self.orientation.transform_vector(v)
    }
}

pub struct Kinematics {
    frame_anchors: Vec<Vector3<f32>>,
    stylus_anchors: Vec<Vector3<f32>>,
    pub size: [f32; 3],
}

impl Kinematics {
    pub fn new(path: &Path) -> Kinematics {
        let mut buf = String::new();
        let mut f = File::open(path).expect("Unable to open robot configuration");
        f.read_to_string(&mut buf)
            .expect("Unable to read robot configuration");

        let rc: RobotConfiguration =
            serde_json::from_str(&buf).expect("Unable to parse robot configuration");

        Kinematics {
            frame_anchors: rc
                .frame_anchors
                .iter()
                .map(|&[x, y, z]| Vector3::new(x, y, z))
                .collect::<Vec<_>>(),
            stylus_anchors: rc
                .stylus_anchors
                .iter()
                .map(|&[x, y, z]| Vector3::new(x, y, z))
                .collect::<Vec<_>>(),
            size: rc.stylus_dims,
        }
    }

    pub fn num_cables(&self) -> usize {
        self.frame_anchors.len()
    }

    pub fn forward(&self, _cable_lengths: &Vec<f32>, _previous_pose: &Pose) -> Pose {
        Pose::new()
    }

    pub fn inverse(&self, pose: &Pose) -> Vec<f32> {
        self.stylus_anchors
            .iter()
            .map(|a| pose.transform_vector(a))
            .zip(self.frame_anchors.iter())
            .map(|(s, &f)| (f - s).norm())
            .collect::<Vec<_>>()
    }

    pub fn cable_ends(&self, pose: &Pose) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        self.stylus_anchors
            .iter()
            .map(|a| pose.transform_vector(a))
            .zip(self.frame_anchors.iter().cloned())
            .collect::<Vec<_>>()
    }
}
