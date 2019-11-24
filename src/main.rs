extern crate autodiff;
extern crate kiss3d;
extern crate nalgebra as na;
extern crate rand;
extern crate rand_distr;
extern crate serde;
extern crate serde_json;

use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Point3, Translation3, Unit, UnitQuaternion, Vector3};

use rand::thread_rng;
use rand_distr::{Distribution, Normal};

use std::path::Path;
use std::time::Instant;

mod kinematics;
mod plot;

use kinematics::{Kinematics, Pose};
use plot::Plot;

const NOISE_BIAS: f32 = 0.0;
const NOISE_SCALE: f32 = 0.0;

fn main() {
    let mut rng = thread_rng();
    let error_mean_distr = Normal::new(0.0, NOISE_BIAS).unwrap();
    let error_std_distr = Normal::new(0.0, NOISE_SCALE).unwrap();
    let small_rand = Normal::new(0.0, 0.01).unwrap();

    // initialize kinematic model from a configuration file
    let kin = Kinematics::new(&Path::new("robot_conf.json"));
    let mut pose = Pose::new();

    // initialize window and graphics primitives
    let mut window = Window::new("Parallel Cable Robot Simulation");

    window.set_light(Light::StickToCamera);

    let mut c = window.add_cube(kin.stylus_size[0], kin.stylus_size[1], kin.stylus_size[2]);
    c.set_color(1.0, 0.0, 0.0);
    let sf = 1.02;
    let mut est = window.add_cube(
        sf * kin.stylus_size[0],
        sf * kin.stylus_size[1],
        sf * kin.stylus_size[2],
    );
    est.set_color(0.5, 0.5, 1.0);
    est.set_lines_width(4.0);
    est.set_surface_rendering_activation(false);

    // initialize the plotting structs
    let data_points = 800;

    let mut cable_length_plots = (0..(kin.num_cables()))
        .map(|_| Plot::new(data_points))
        .collect::<Vec<_>>();

    let mut cable_length_noisy_plots = (0..(kin.num_cables()))
        .map(|_| Plot::new(data_points))
        .collect::<Vec<_>>();

    let mut frame_time_plot = Plot::new(data_points);
    let error_distrs = (0..(kin.num_cables()))
        .map(|_| {
            Normal::new(
                error_mean_distr.sample(&mut rng),
                (error_std_distr.sample(&mut rng) as f32).abs(),
            )
            .unwrap()
        })
        .collect::<Vec<_>>();

    let mut time = Instant::now();
    let mut t = 0.0f32;
    let dt = 0.01f32;

    // last estimated pose. We start from a known location
    let mut est_pose = Pose::new();
    est_pose.orientation = UnitQuaternion::new(Vector3::new(
        small_rand.sample(&mut rng),
        small_rand.sample(&mut rng),
        small_rand.sample(&mut rng),
    ));

    // main render loop
    while window.render() {
        // make sure we know the current size of the screen in case of resizes
        let window_w = window.width();
        let window_h = window.height();

        // create a new pose, simulating movement
        let x = 0.25 * t.sin();
        let y = 0.25 * t.cos();
        let z = 0.125 * (4. * t).sin();

        pose.position = Vector3::new(x, y, z);
        pose.orientation =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vector3::new(x, y, z)), 0.5);

        let cable_lengths = kin.inverse(&pose);

        // add error to the cable lengths
        let cable_lengths_noisy = cable_lengths
            .iter()
            .enumerate()
            .map(|(i, &l)| l + error_distrs[i].sample(&mut rng))
            .collect::<Vec<_>>();

        // update estimated pose with the new (noisy) cable lengths
        est_pose = kin.forward(&cable_lengths_noisy, &est_pose);

        // update the visualization
        c.set_local_rotation(pose.orientation);
        c.set_local_translation(Translation3::from(pose.position));

        est.set_local_rotation(est_pose.orientation);
        est.set_local_translation(Translation3::from(est_pose.position));

        // draw the cables from the true pose
        for (p1, p2) in kin.cable_segments(&pose) {
            window.draw_line(
                &Point3::from(p1),
                &Point3::from(p2),
                &Point3::new(1.0, 1.0, 1.0),
            );
        }
        // draw the cables from the estimated pose
        for (p1, p2) in kin.cable_segments(&est_pose) {
            window.draw_line(
                &Point3::from(p1),
                &Point3::from(p2),
                &Point3::new(0.5, 0.5, 1.0),
            );
        }
        // update and draw cable length plots
        for (i, &l) in cable_lengths.iter().enumerate() {
            cable_length_plots[i].add_point(l);
            cable_length_plots[i].draw(
                &mut window,
                &Point3::new(0.0, 1.0, 0.0),
                -0.25 * (window_w as f32),
                -0.25 * (window_h as f32),
                window_w as f32 / data_points as f32 * 0.5,
                200.,
            );
            cable_length_noisy_plots[i].add_point(cable_lengths_noisy[i]);
            cable_length_noisy_plots[i].draw(
                &mut window,
                &Point3::new(0.5, 0.5, 1.0),
                -0.25 * (window_w as f32),
                -0.25 * (window_h as f32),
                window_w as f32 / data_points as f32 * 0.5,
                200.,
            );
        }

        // get and display framerate diagnostic information
        t += dt;
        let new_time = Instant::now();
        let frame_time = new_time.duration_since(time).subsec_nanos() as f32 / 1.0e6;
        time = new_time;

        frame_time_plot.add_point(frame_time);

        frame_time_plot.draw(
            &mut window,
            &Point3::new(1.0, 0.0, 0.0),
            -0.25 * (window_w as f32),
            -0.25 * (window_h as f32),
            window_w as f32 / data_points as f32 * 0.5,
            1.0,
        );
    }
}
