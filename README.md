<div align="center">
  <center><img width="220" src="https://raw.githubusercontent.com/pantor/frankx/master/doc/logo.png"></div></center>
  <h3 align="center"><center><i>frankx</i><br>High-Level Motion Library for the Franka Panda Robot</center></h3>
</div>

<p align="center">
  <a href="https://github.com/pantor/frankx/actions">
    <img src="https://github.com/pantor/frankx/workflows/CI/badge.svg" alt="CI">
  </a>

  <a href="https://github.com/pantor/frankx/issues">
    <img src="https://img.shields.io/github/issues/pantor/frankx.svg" alt="Issues">
  </a>

  <a href="https://github.com/pantor/frankx/releases">
    <img src="https://img.shields.io/github/release/pantor/frankx.svg" alt="Releases">
  </a>

  <a href="https://github.com/pantor/frankx/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-LGPL-green.svg" alt="LGPL">
  </a>
</p>

Frankx is a high-level motion library (both C++ and Python) for the Franka Emika Panda robot. It is loosely based on the KUKA Sunrise.OS API for the LBR iiwa.


## Installation

Frankx is based on [libfranka](https://frankaemika.github.io/docs/libfranka.html) (v.0.6.0), [Reflexxes](http://reflexxes.ws) as a real-time trajectory-generator, [Eigen](https://eigen.tuxfamily.org) (v3.3.7) for transformation calculations and [pybind11](https://github.com/pybind/pybind11) (v2.4.0) for the Python bindings. Make sure to have these dependencies installed, then you can install frankx via

```bash
mkdir -p build
cd build
cmake -DReflexxes_ROOT_DIR=../RMLTypeII -DREFLEXXES_TYPE=ReflexxesTypeII -DBUILD_TYPE=Release ..
make -j4
make install
```

Of course, you need to adapt the Reflexxes directory and type (either `ReflexxesTypeII` or `ReflexxesTypeIV`). You can also include frankx as a subproject in your parent CMake via `add_subdirectory(frankx)` and then `target_link_libraries(your_target frankx)`.


## Tutorial

In your C++ project, just include `frankx/frankx.hpp` and link the library. As a very basic example, only four lines of code are needed for simple robotic motions.

```c++
#include <frankx/frankx.hpp>
using namespace frankx;

// Connect to the robot with the FCI IP address
Robot robot("172.16.0.2");

// Reduce velocity and acceleration of the robot
robot.setDynamicRel(0.05);

// Move the end-effector 20cm in positive x-direction
auto motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0));

// Finally move the robot
robot.move(motion);
```

Or the corresponding program in python
```python
from frankx import Affine, LinearRelativeMotion, Robot

robot = Robot("172.16.0.2")
robot.set_dynamic_rel(0.05)

motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0))
robot.move(motion)
```


### Geometry

`frankx::Affine` is a thin wrapper around [Eigen::Affine3d](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html). It is used for Cartesian poses, frames and transformation. Frankx simplifies the usage of Euler angles (default ZYX-convention).
```c++
// Initiliaze a transformation with an (x, y, z) translation
Affine z_translation = Affine(0.0, 0.0, 0.5);

// Define a rotation transformation using the (x, y, z, a, b, c) parameter list
Affine z_rotation = Affine(0.0, 0.0, 0.0, 0.1, 0.0, 0.0);

// Make use of the wonderful Eigen library
auto combined_transformation = z_translation * z_rotation;

// Get the Euler angles (a, b, c) in a vector representation
Eigen::Vector3d euler_angles = combined_transformation.angles();

// Get the vector representation (x, y, z, a, b, c) of an affine transformation
frankx::Vector6d pose = combined_transformation.vector();
```
In all cases, distances are in [m] and rotations in [rad]. Additionally, there are several helper functions for conversion between Eigen and libfranka's std::array objects.


### Motion Types

Apply a motion to the end effector.

- LinearMotion
- LinearRelativeMotion
- WaypointMotion
- PositionHold

Apply a motion to a given frame.

```c++
Affine camera_frame = Affine(0.1, 0.0, 0.1);
robot.move(camera_frame, motion);
```

Change dynamic limits of a specific motion, multiplied with robot velocity_rel and acceleration_rel.
```c++
robot.setVelocityRel(2.0).setAccelerationRel(1.0);
robot.setJerkRel(0.1); // Only if you use the Reflexxes Type IV library
```


### Reaction Conditions

- SpatialForceBreakCondition
- NormalForceBreakCondition
- TimeBreakCondition

If the robot is pushed, the program continues its execution.

```c++
// Stop motion if force is over 10N
auto data = MotionData().withCondition({
  Condition::Measure::ForceXYZNorm, Condition::Comparison::Greater, 10.0  // [N]
});

// Hold position for 5s
robot.move(PositionHold(5.0), data); // [s]
```


### Real-time Waypoint Motion

```c++
robot.moveAsync(motion_hold);

// Wait for key input from user
std::cin.get()

motion_hold.setNextWaypoint(const Waypoint& waypoint);
```


### Gripper

In the `frankx::Gripper` class, the default gripper force and gripper speed can be set. Then, the following simplified commands can be used:

```c++
auto gripper = Gripper("172.16.0.2");

// These are the default values
gripper.gripper_speed = 0.1; // [m/s]
gripper.gripper_force = 30.0; // [N]

// Preshape gripper before grasp, use the given speed
gripper.move(50.0); // [mm]

// Grasp an object
gripper.clamp();

// Release an object and move to a given distance
gripper.release(50.0);
```


## Examples

Multiple examples can be found in `examples`.


## Development

Frankx is written in C++17 and Python3. It works well with ROS2.


## License

For non-commercial applications, this software is licensed under the LGPL v3.0. If you want to use frankx within commercial applications or under a different license, please contact us for individual agreements.
