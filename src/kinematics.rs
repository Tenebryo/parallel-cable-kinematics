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

        // iterate to get angle
        let mut off_pulley = direct;
        let mut angle_est = 0.0;

        for _ in 0..4 {
            let angle_rotation =
                UnitQuaternion::from_axis_angle(&Unit::new_normalize(pulley_axis), -angle_est);
            let new_point = pivot + pulley_center - angle_rotation.transform_vector(&pulley_center);
            // compute vector from pulley point to end point
            off_pulley = end - new_point;
            angle_est = off_pulley.angle(&axis);
        }

        // calculate the length of the wire
        let len = off_pulley.norm() + angle_est * rad;

        // generate line segments that make up the wire length
        let mut seg = Vec::with_capacity(8);
        let mut last_point = pivot;
        for i in 0..7 {
            let rot = UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(pulley_axis),
                i as f32 * -angle_est / 7.0,
            );
            let new_point = pivot + pulley_center - rot.transform_vector(&pulley_center);
            seg.push((last_point, new_point));
            last_point = new_point;
        }
        seg.push((last_point, end));

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