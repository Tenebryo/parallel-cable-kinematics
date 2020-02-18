# Parallel Cable Kinematics
Forward and inverse kinematics for an cable-driven parallel robot. 
The kinematics assume that the cables are guided by pulleys at each of the stationary anchors.

## Installation and Usage

This proof of concept is made using Rust. To run the code, make sure an up-to-date version of cargo is installed (this is most easily achieved via [https://rustup.rs/] if you do not already have it), and run `cargo run --release` in the top level directory of this repository. This should download and install all the Rust crate dependencies, compile the source, and run the project.

## Terms

 * Frame: stationary portion of the robot.
 * End Effector: the part of the robot that we desire to control or move around.
 * Anchor: a point where the cable interfaces with the frame or the end effector.\
 * Pose: A translation and orientation of the end effector

## Details

The true pose of the robot is displayed as white cables with a red rectangular prism as the end effector. 
The blue cables and wireframe end effector represent the estimated pose.

The exact configuration of how the cables are attached can be configured in the [robot_conf.json](robot_conf.json) file. 
The `"cables"` list is a list of triples of 3D vectors; the global coordinates of the pivot of the pulley guide, 
the axis the pulley guide rotates around (the length of which doubles as the pulley radius), 
and the coordinates of the cable attachment point on the end effector (relative to the end effector center). 
The `"stylus_dims"` list is a vector that describes the dimensions of the end effector to render.

## Plot Legend

Simple scanning plots are used to display certain values over time:

 * Green (100px/unit): True cable lengths
 * Blue (100px/unit): Noisy cable lengths (simulating measurement error)
 * Red (1px/unit): Milliseconds per frame
 * Purple (10000px/unit): translational error
 * Yellow (100px/unit): rotational error
 * Cyan (10000px/unit): total error (sum of translational error and 0.01 times the rotational error so they are comparable)

## Kinematics

The kinematics of a cable-driven parallel robot are interesting because they differ from the kinematics of most robots (serial robots like an arm) in how the kinematics are calculated. In a serial robot, the forward kinematics are simple to calculate with some basic matrix and vector operations, but the inverse kinematics solution is more involved. In a parallel robot, this is reversed, as it is simple to calculate the inverse kinematics (the lengths of the cables, in this case), but for the forward kinematics we have to figure out what pose results in the measured cable lengths, which is can be a large non-linear system.

### Inverse Kinematics

Inverse kinematics for the parallel cable robot are relatively simple. We just translate the end effector cable anchors relative to the frame and calculate the distance between the two anchors for each cable. However, this model assumes the wires are simply terminated at points on each end. This isn't unreasonable for the fixed end on the end effector, but if we are using a pivoting pulley system, a point approximation won't be accurate. The cable always stays in the plane of the pulley, so we calculate the point on the pulley that the cable leaves it. This is a small, non-linear calculation (there are a few cases to check to get right) that I solved with Mathematica and implemented it at [src/kinematics.rs#L122]. Once you have that point, the total length around the pulley and to the other anchor can be calculated accurately.

### Forward Kinematics

The forward kinematics for a robot like this doesn't have a general solution for any robot configuration because of the potential for singularities or loose cables, so instead we use gradient descent. We can easily define how far off an estimate is by comparing the calculating the cable lengths for the estimate (using the inverse kinematics) and then comparing them to the the measured lengths. We can combine this into a single error number, which I did with the mean squared error. Ideally, we want this error to be zero, which we can accomplish with gradient descent. By calculating the gradient of the error with respect to the pose (translation and rotation), we can see how to adjust the parameters in order reduce the total error. We repeatedly tweek the parameters in the right direction, little by little, either for a fixed number of iterations or until the error is small enough. Once the error is low enough, we return the pose represented by the final parameters.