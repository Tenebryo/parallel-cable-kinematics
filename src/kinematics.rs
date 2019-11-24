use na::{Unit, UnitQuaternion, Vector3};
use serde::Deserialize;
use serde_json;

use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

/// Simple struct to handle deserialization of a configuration file
#[derive(Deserialize)]
struct RobotConfiguration {
    /// [[pivot], [axis], [end]]
    cables: Vec<[[f32; 3]; 3]>,
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

/// Represents a cable system including the pivoting pulley
pub struct Cable {
    pivot: Vector3<f32>,
    axis: Vector3<f32>,
    end: Vector3<f32>,
}

impl Cable {
    pub fn new(pivot: Vector3<f32>, axis: Vector3<f32>, end: Vector3<f32>) -> Cable {
        Cable { pivot, axis, end }
    }

    /// returns (<length>, <segments>)
    fn calculate_pulley_state(&self, pose: &Pose) -> (f32, Vec<(Vector3<f32>, Vector3<f32>)>) {

        let end = pose.transform_vector(&self.end);
        let axis = self.axis;
        let pivot = self.pivot;

        let rad = axis.norm();

        // move pivot to origin
        let direct = end - pivot;
        // project onto the plane defined by axis
        let planar = direct - (direct.dot(&axis) / (rad * rad)) * axis;

        // scale planar vector to the pulley radius to get the pulley center relative to origin.
        let pulley_center = (rad / planar.norm()) * planar;

        // Get the axis that the pulley rotates around
        let pulley_axis = pulley_center.cross(&axis);

        let center_to_end = end - (pivot + pulley_center);

        // Find explicit solution for the tangent point in the 2D plane of the pulley
        let ex = pulley_center.dot(&center_to_end) / rad;
        let ey = axis.dot(&center_to_end) / rad;

        let x = {
            let r2 = rad * rad;
            let ex2 = ex * ex;
            let ey2 = ey * ey;
            let i0 = ex * r2;
            let i1 = (ey2 * r2 * (ex2 + ey2 - r2)).sqrt();
            let i2 = ex2 + ey2;

            if ey >= 0.0 {
                (i0 - i1) / i2
            } else {
                (i0 + i1) / i2
            }
        };

        let y = ex.signum() * (rad * rad - x * x).sqrt();

        // calculate the pulley contact angle and the tangent point in 3D space
        let angle_est = y.atan2(-x);

        let tangent_point = pivot + (1.0 + x / rad) * pulley_center + (y / rad) * axis;

        // calculate the length of the wire
        let len = (end - tangent_point).norm() + angle_est * rad;


        // generate line segments that make up the wire length
        const NUM_PULLEY_SEGMENTS : usize = 7;

        let mut seg = Vec::with_capacity(NUM_PULLEY_SEGMENTS + 1);
        let mut last_point = pivot;

        for i in 0..NUM_PULLEY_SEGMENTS {
            let rot = UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(pulley_axis),
                i as f32 * -angle_est / (NUM_PULLEY_SEGMENTS as f32 - 1.0),
            );
            let new_point = pivot + pulley_center - rot.transform_vector(&pulley_center);
            seg.push((last_point, new_point));
            last_point = new_point;
        }
        seg.push((tangent_point, end));

        (len, seg)
    }

    /// computes the length of the cable including the bit around the pulley
    pub fn len(&self, pose: &Pose) -> f32 {
        self.calculate_pulley_state(pose).0
    }

    pub fn segments(&self, pose: &Pose) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        self.calculate_pulley_state(pose).1
    }
}

/// struct that computes 
pub struct Kinematics {
    cables : Vec<Cable>,
    pub stylus_size: [f32; 3],
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
            cables : rc.cables.iter()
                .map(|&[[px, py, pz], [ax, ay, az], [ex, ey, ez]]| Cable::new(
                    Vector3::new(px, py, pz),
                    Vector3::new(ax, ay, az),
                    Vector3::new(ex, ey, ez)
                ))
                .collect::<Vec<_>>(),
            stylus_size: rc.stylus_dims,
        }
    }

    pub fn num_cables(&self) -> usize {
        self.cables.len()
    }

    pub fn forward(&self, _cable_lengths: &Vec<f32>, _previous_pose: &Pose) -> Pose {
        Pose::new()
    }

    pub fn inverse(&self, pose: &Pose) -> Vec<f32> {
        self.cables.iter().map(|c| c.len(pose)).collect()
    }

    pub fn cable_segments(&self, pose: &Pose) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        self.cables.iter().map(|c| c.segments(pose)).flatten().collect::<Vec<_>>()
    }
}