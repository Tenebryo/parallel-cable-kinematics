# parallel-cable-kinematics
Forward and inverse kinematics for an cable-driven parallel robot. 
The kinematics assume that the cables are guided by pulleys at each of the stationary anchors.

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
