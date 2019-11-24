use autodiff::{grad, Float, F};
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

    #[allow(dead_code)]
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
    pub fn calculate_pulley_state(
        &self,
        tx: F,
        ty: F,
        tz: F,
        ux: F,
        uy: F,
        uz: F,
    ) -> (F, Vec<(Vector3<f32>, Vector3<f32>)>) {
        let end = {
            let end = Vector3::new(
                F::cst(self.end[0]),
                F::cst(self.end[1]),
                F::cst(self.end[2]),
            );

            let th = (ux * ux + uy * uy + uz * uz).sqrt();

            let cos = th.cos();
            let sin = th.sin();
            let one = F::cst(1.0);

            let ux = ux / th;
            let uy = uy / th;
            let uz = uz / th;

            Vector3::new(
                tx + (cos + ux * ux * (one - cos)) * end[0]
                    + (ux * uy * (one - cos) - uz * sin) * end[1]
                    + (ux * uz * (one - cos) + uy * sin) * end[2],
                ty + (uy * ux * (one - cos) + uz * sin) * end[0]
                    + (cos + uy * uy * (one - cos)) * end[1]
                    + (uy * uz * (one - cos) - ux * sin) * end[2],
                tz + (uz * ux * (one - cos) - uy * sin) * end[0]
                    + (uz * uy * (one - cos) + ux * sin) * end[1]
                    + (cos + uz * uz * (one - cos)) * end[2],
            )
        };
        let axis = Vector3::new(
            F::cst(self.axis[0]),
            F::cst(self.axis[1]),
            F::cst(self.axis[2]),
        );
        let pivot = Vector3::new(
            F::cst(self.pivot[0]),
            F::cst(self.pivot[1]),
            F::cst(self.pivot[2]),
        );

        let rad = axis.dot(&axis).sqrt();

        // move pivot to origin
        let direct = end - pivot;
        // project onto the plane defined by axis
        let planar = direct - axis * (direct.dot(&axis) / (rad * rad));

        // scale planar vector to the pulley radius to get the pulley center relative to origin.
        let pulley_center = planar * (rad / planar.dot(&planar).sqrt());

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

            if ey >= F::cst(0.0) {
                (i0 - i1) / i2
            } else {
                (i0 + i1) / i2
            }
        };

        let y = ex.signum() * (rad * rad - x * x).sqrt();

        // calculate the pulley contact angle and the tangent point in 3D space
        let angle = y.atan2(-x);
        let tangent_point : Vector3<F> = pivot + (pulley_center * (1.0 + x / rad)) + (axis * (y / rad));

        // calculate the length of the wire
        let straight_wire = end - tangent_point;
        let len = straight_wire.dot(&straight_wire).sqrt() + angle * rad;

        // generate line segments that make up the wire length
        const NUM_PULLEY_SEGMENTS: usize = 7;

        // Get the axis that the pulley rotates around
        let pulley_center = Vector3::new(
            pulley_center[0].x as f32,
            pulley_center[1].x as f32,
            pulley_center[2].x as f32,
        );
        let pulley_axis = pulley_center.cross(&self.axis);

        let mut seg = Vec::with_capacity(NUM_PULLEY_SEGMENTS + 1);
        let mut last_point = self.pivot;

        let angle = angle.x as f32;

        for i in 0..NUM_PULLEY_SEGMENTS {
            let rot = UnitQuaternion::from_axis_angle(
                &Unit::new_normalize(pulley_axis),
                i as f32 * -angle / (NUM_PULLEY_SEGMENTS as f32 - 1.0),
            );
            let new_point = self.pivot + pulley_center - rot.transform_vector(&pulley_center);
            seg.push((last_point, new_point));
            last_point = new_point;
        }
        seg.push((last_point, Vector3::new(
            end[0].x as f32,
            end[1].x as f32,
            end[2].x as f32,
        )));

        (len, seg)
    }

    /// computes the length of the cable including the bit around the pulley
    pub fn len(&self, pose: &Pose) -> f32 {
        let tx = F::cst(pose.position[0]);
        let ty = F::cst(pose.position[1]);
        let tz = F::cst(pose.position[2]);

        let axis = pose.orientation.scaled_axis();

        let ux = F::cst(axis[0]);
        let uy = F::cst(axis[1]);
        let uz = F::cst(axis[2]);

        self.calculate_pulley_state(tx, ty, tz, ux, uy, uz).0.x as f32
    }

    pub fn segments(&self, pose: &Pose) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        let tx = F::cst(pose.position[0]);
        let ty = F::cst(pose.position[1]);
        let tz = F::cst(pose.position[2]);

        let axis = pose.orientation.scaled_axis();

        let ux = F::cst(axis[0]);
        let uy = F::cst(axis[1]);
        let uz = F::cst(axis[2]);

        self.calculate_pulley_state(tx, ty, tz, ux, uy, uz).1
    }
}

/// struct that computes
pub struct Kinematics {
    cables: Vec<Cable>,
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
            cables: rc
                .cables
                .iter()
                .map(|&[[px, py, pz], [ax, ay, az], [ex, ey, ez]]| {
                    Cable::new(
                        Vector3::new(px, py, pz),
                        Vector3::new(ax, ay, az),
                        Vector3::new(ex, ey, ez),
                    )
                })
                .collect::<Vec<_>>(),
            stylus_size: rc.stylus_dims,
        }
    }

    pub fn num_cables(&self) -> usize {
        self.cables.len()
    }

    pub fn forward(&self, _cable_lengths: &Vec<f32>, _previous_pose: &Pose) -> Pose {
        const SGD_ROUNDS : usize = 32;
        let mut rate = 0.1;

        let mut pose = Vec::with_capacity(6);
        pose.push(_previous_pose.position[0] as f64);
        pose.push(_previous_pose.position[1] as f64);
        pose.push(_previous_pose.position[2] as f64);

        let axis = _previous_pose.orientation.scaled_axis();

        pose.push(1.0);
        pose.push(axis[0] as f64);
        pose.push(axis[1] as f64);
        pose.push(axis[2] as f64);

        for _ in 0..SGD_ROUNDS {
            let error_func = |l : &[F]| {
                if let &[tx, ty, tz, qw, qx, qy, qz] = l {

                    let mag = (qw*qw + qx*qx + qy*qy + qz*qz).sqrt();
                    
                    let ux = qx / mag;
                    let uy = qy / mag;
                    let uz = qz / mag;


                    let mut loss = F::cst(0.0);
                    for (i,c) in self.cables.iter().enumerate() {
                        let err = c.calculate_pulley_state(tx, ty, tz, ux, uy, uz).0 - F::cst(_cable_lengths[i]);
                        loss += err * err;
                    }

                    loss
                } else {
                    F::cst(0.0)
                }
            };

            let g = grad(error_func, &pose);

            for i in 0..7 {
                pose[i] -= if i >= 3 {500.} else {1.} * rate * g[i];
            }

            rate *= 0.9;
        }
        let mut r = Pose::new();
        r.position = Vector3::new(pose[0] as f32, pose[1] as f32, pose[2] as f32);
        r.orientation = UnitQuaternion::new(Vector3::new(pose[4] as f32, pose[5] as f32, pose[6] as f32));

        r
    }

    pub fn inverse(&self, pose: &Pose) -> Vec<f32> {
        self.cables.iter().map(|c| c.len(pose)).collect()
    }

    pub fn cable_segments(&self, pose: &Pose) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        self.cables
            .iter()
            .map(|c| c.segments(pose))
            .flatten()
            .collect::<Vec<_>>()
    }
}
