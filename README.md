<div align="center">
  <img width="340" src="https://raw.githubusercontent.com/pantor/frankx/master/doc/logo.svg?sanitize=true">
  <h3 align="center">
    High-Level Motion Library for the Franka Emika Robot
  </h3>
</div>
<p align="center">
  <a href="https://github.com/pantor/frankx/actions">
    <img src="https://github.com/pantor/frankx/workflows/CI/badge.svg" alt="CI">
  </a>

  <a href="https://github.com/pantor/frankx/actions">
    <img src="https://github.com/pantor/frankx/workflows/Publish/badge.svg" alt="Publish">
  </a>

  <a href="https://github.com/pantor/frankx/issues">
    <img src="https://img.shields.io/github/issues/pantor/frankx.svg" alt="Issues">
  </a>

  <a href="https://github.com/pantor/frankx/releases">
    <img src="https://img.shields.io/github/v/release/pantor/frankx.svg?include_prereleases&sort=semver" alt="Releases">
  </a>

  <a href="https://github.com/pantor/frankx/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-LGPL-green.svg" alt="LGPL">
  </a>
</p>


Frankx is a high-level motion library (both C++ and Python) for the Franka Emika robot. It adds a Python wrapper around [libfranka](https://frankaemika.github.io/docs/libfranka.html), while replacing necessary real-time programming with higher-level motion commands. As frankx focuses on making real-time trajectory generation easy, it allows the robot to react to unforeseen events.


## Installation

To start using frankx with Python and libfranka *0.9.0*, just install it via
```bash
pip install frankx
```

Frankx is based on [libfranka](https://github.com/frankaemika/libfranka), [Eigen](https://eigen.tuxfamily.org) for transformation calculations and [pybind11](https://github.com/pybind/pybind11) for the Python bindings. Frankx uses the [Ruckig](https://ruckig.com) Community Version for Online Trajectory Generation (OTG). As the Franka is quite sensitive to acceleration discontinuities, it requires constrained jerk for all motions. After installing the dependencies (the exact versions can be found below), you can build and install frankx via

```bash
git clone --recurse-submodules git@github.com:pantor/frankx.git
cd frankx
mkdir -p build
cd build
cmake -DBUILD_TYPE=Release ..
make
make install
```

To use frankx, you can also include it as a subproject in your parent CMake via `add_subdirectory(frankx)` and then `target_link_libraries(<target> libfrankx)`. If you need only the Python module, you can install frankx via

```bash
pip install .
```

Make sure that the built library can be found from Python by adapting your Python Path.


#### Using Docker

To use frankx within Docker we have supplied a [Dockerfile](docker/Dockerfile) which you currently need to build yourself:

```bash
git clone https://github.com/pantor/frankx.git
cd frankx/
docker build -t pantor/frankx --build-arg libfranka_version=0.7.0 -f docker/Dockerfile .
```

To use another version of libfranka than the default (v.0.7.0) simply change the build argument. Then, to run the container simply:

```bash
docker run -it --rm --network=host --privileged pantor/frankx
```

The container requires access to the host machines network *and* elevated user rights to allow the docker user to set RT capabilities of the processes run from within it.


## Tutorial

Frankx comes with both a C++ and Python API that differ only regarding real-time capability. We will introduce both languages next to each other. In your C++ project, just include `include <frankx/frankx.hpp>` and link the library. For Python, just `import frankx`. As a first example, only four lines of code are needed for simple robotic motions.

```.cpp
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

The corresponding program in Python is
```.py
from frankx import Affine, LinearRelativeMotion, Robot

robot = Robot("172.16.0.2")
robot.set_dynamic_rel(0.05)

motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0))
robot.move(motion)
```

Furthermore, we will introduce methods for geometric calculations, for moving the robot according to different motion types, how to implement real-time reactions and changing waypoints in real time as well as controlling the gripper.


### Geometry

`frankx::Affine` is a thin wrapper around [Eigen::Affine3d](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html). It is used for Cartesian poses, frames and transformation. Frankx simplifies the usage of Euler angles (default ZYX-convention).
```.cpp
// Initiliaze a transformation with an (x, y, z, a=0.0, b=0.0, c=0.0) translation
Affine z_translation = Affine(0.0, 0.0, 0.5);

// Define a rotation transformation using the (x, y, z, a, b, c) parameter list
Affine z_rotation = Affine(0.0, 0.0, 0.0, M_PI / 3, 0.0, 0.0);

// Make use of the wonderful Eigen library
auto combined_transformation = z_translation * z_rotation;

// Get the Euler angles (a, b, c) in a vector representation
Eigen::Vector3d euler_angles = combined_transformation.angles();

// Get the vector representation (x, y, z, a, b, c) of an affine transformation
frankx::Vector6d pose = combined_transformation.vector();
```

In all cases, distances are in [m] and rotations in [rad]. Additionally, there are several helper functions for conversion between Eigen and libfranka's std::array objects. In python, this translates into
```.py
z_translation = Affine(0.0, 0.0, 0.5)
z_rotation = Affine(0.0, 0.0, 0.0, math.pi / 3, 0.0, 0.0)
combined_transformation = z_translation * z_rotation

# These two are now numpy arrays
euler_angles = combined_transformation.angles()
pose = combined_transformation.vector()
```

As the trajectory generation works in the Euler space, please make sure to have continuous Euler angles around your working point. You can adapt this by setting the flange to end-effector transformation via `setEE(...)`.


### Robot

We wrapped most of the libfanka API (including the RobotState or ErrorMessage) for Python. Moreover, we added methods to adapt the dynamics of the robot for all motions. The `rel` name denotes that this a factor of the maximum constraints of the robot.
```.py
robot = Robot("172.16.0.2")

# Recover from errors
robot.recover_from_errors()

# Set velocity, acceleration and jerk to 5% of the maximum
robot.set_dynamic_rel(0.05)

# Alternatively, you can define each constraint individually
robot.velocity_rel = 0.2
robot.acceleration_rel = 0.1
robot.jerk_rel = 0.01

# Get the current pose
current_pose = robot.current_pose()
```


### Robot State

The robot state can be retrieved by calling the following methods:

* `get_state`: Return an object of the `frankx.RobotState` class which contains most of the same attributes, under the same name, as the libfranka [franka::RobotState](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html) definition.

* `current_pose`: Return a 3D Affine transformation object of the measured end effector pose in base frame (alias for [franka::RobotState::O_T_EE](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#a193781d47722b32925e0ea7ac415f442)) affected or not by another Affine transformation.

* `current_joint_positions`: Return a sequence of the manipulator arm's 7-joint positions (alias for [franka::RobotState::q](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#ade3335d1ac2f6c44741a916d565f7091)).

```.py
robot = Robot("172.16.0.2")

# Get the current state
state = robot.get_state()
pose = robot.current_pose()
joint_positions = robot.current_joint_positions()
```

However, invoking these methods as-is throws an exception (`_frankx.InvalidOperationException: libfranka robot: Cannot perform this operation while another control or read operation is running`) when the robot is moving. In this case, the methods must be invoked by setting the `read_once` argument to False (default to True) to retrieve the state.
```.py
robot = Robot("172.16.0.2")

# Get the current state handling the read exception when the robot is in motion
try:
    robot_state = robot.get_state(read_once=True)
except frankx.InvalidOperationException:
    robot_state = robot.get_state(read_once=False)

# Get the current pose handling the read exception when the robot is in motion
try:
    pose = robot.current_pose(read_once=True)
except frankx.InvalidOperationException:
    pose = robot.current_pose(read_once=False)

# Get the current joint positions handling the read exception when the robot is in motion
try:
    joint_positions = robot.current_joint_positions(read_once=True)
except frankx.InvalidOperationException:
    joint_positions = robot.current_joint_positions(read_once=False)
```


### Motion Types

Frankx defines five different motion types. In python, you can use them as follows:
```.py
# A point-to-point motion in the joint space
m1 = JointMotion([-1.81194, 1.17910, 1.75710, -2.1416, -1.14336, 1.63304, -0.43217])

# A linear motion in cartesian space
m2 = LinearMotion(Affine(0.2, -0.4, 0.3, math.pi / 2, 0.0, 0.0))
m3 = LinearMotion(Affine(0.2, -0.4, 0.3, math.pi / 2, 0.0, 0.0), elbow=1.7)  # With target elbow angle

# A linear motion in cartesian space relative to the initial position
m4 = LinearRelativeMotion(Affine(0.0, 0.1, 0.0))

# A more complex motion by defining multiple waypoints
m5 = WaypointMotion([
  Waypoint(Affine(0.2, -0.4, 0.2, 0.3, 0.2, 0.1)),
  # The following waypoint is relative to the prior one
  Waypoint(Affine(0.0, 0.1, 0.0), Waypoint.ReferenceType.Relative)
])

# Hold the position for [s]
m6 = PositionHold(5.0)
```

The real robot can be moved by applying a motion to the robot using `move`:
```.py
robot.move(m1)
robot.move(m2)

# To use a given frame relative to the end effector
camera_frame = Affine(0.1, 0.0, 0.1)
robot.move(camera_frame, m3)

# To change the dynamics of the motion, use MotionData
data = MotionData(0.2)  # Using a dynamic_rel of 0.2 (eventually multiplied with robot.dynamic_rel)
robot.move(m4, data)
```

Using MotionData, you can adapt the dynamics (velocity, acceleration and jerk) of a specific motion.
```.py
data.velocity_rel = 1.0
data.jerk_rel = 0.2
```


### Real-Time Reactions

By adding reactions to the motion data, the robot can react to unforeseen events. In the Python API, you can define conditions by using a comparison between a robot's value and a given threshold. If the threshold is exceeded, the reaction fires. Following comparisons are currently implemented
```.py
reaction_motion = LinearRelativeMotion(Affine(0.0, 0.0, 0.01))  # Move up for 1cm

# Stop motion if the overall force is greater than 30N
d1 = MotionData().with_reaction(Reaction(Measure.ForceXYZNorm() > 30.0))

# Apply reaction motion if the force in negative z-direction is greater than 10N
d2 = MotionData().with_reaction(Reaction(Measure.ForceZ() < -10.0), reaction_motion)

# Stop motion if its duration is above 30s
d3 = MotionData().with_reaction(Reaction(Measure.Time() >= 30.0))

robot.move(m2, d2)

# Check if the reaction was triggered
if d2.has_fired:
  robot.recover_from_errors()
  print('Force exceeded 10N!')
```

Once a reaction has fired, it will be neglected furthermore. In C++ you can additionally use lambdas to define more complex behaviours:
```.cpp
// Stop motion if force is over 10N
auto data = MotionData()
  .withReaction({
    Measure::ForceXYZNorm() > 10.0  // [N]
  })
  .withReaction({
    [](const franka::RobotState& state, double time) {
      return (state.current_errors.self_collision_avoidance_violation);
    }
  });

// Hold position for 5s
robot.move(PositionHold(5.0), data); // [s]
// e.g. combined with a PositionHold, the robot continues its program after pushing the end effector.
```


### Real-Time Waypoint Motion

While the robot moves in a background thread, you can change the waypoints in real-time.
```.cpp
robot.moveAsync(motion_hold);

// Wait for key input from user
std::cin.get();

motion_hold.setNextWaypoint(Waypoint(Affine(0.0, 0.0, 0.1), Waypoint::ReferenceType::Relative);
```


### Gripper

In the `frankx::Gripper` class, the default gripper force and gripper speed can be set. Then, additionally to the libfranka commands, the following helper methods can be used:

```.cpp
auto gripper = Gripper("172.16.0.2");

// These are the default values
gripper.gripper_speed = 0.02; // [m/s]
gripper.gripper_force = 20.0; // [N]

// Preshape gripper before grasp, use the given speed
gripper.move(50.0); // [mm]

// Grasp an object of unknown width
is_grasping = gripper.clamp();

// Do something
is_grasping &= gripper.isGrasping();

// Release an object and move to a given distance
if (is_grasping) {
  gripper.release(50.0);
}
```

The Python API should be very straight-forward for the Gripper class.


### Kinematics

Frankx includes a rudimentary, non-realtime-capable forward and inverse kinematics.

```.py
# Some initial joint configuration
q = [-1.45549, 1.15401, 1.50061, -2.30909, -1.3141, 1.9391, 0.02815]

# Calculate the forward kinematics
x = Affine(Kinematics.forward(q))
print('Current end effector position: ', x)

# Define new target position
x_new = Affine(x=0.1, y=0.0, z=0.0) * x

# Franka has 7 DoFs, so what to do with the remaining Null space?
null_space = NullSpaceHandling(2, 1.4) # Set elbow joint to 1.4

# Inverse kinematic with target, initial joint angles, and Null space configuration
q_new = Kinematics.inverse(x_new.vector(), q, null_space)

print('New position: ', x_new)
print('New joints: ', q_new)
```


## Documentation

An auto-generated documentation can be found at [https://pantor.github.io/frankx/](https://pantor.github.io/frankx/). Moreover, there are multiple examples for both C++ and Python in the [examples](https://github.com/pantor/frankx/tree/master/examples) directory. We will add a more detailed documentation once frankx reaches v1.0.


## Development

Frankx is written in C++17 and Python3.7. It is currently tested against following versions

- Eigen v3.3.7
- Libfranka v0.9.0
- Pybind11 v2.9.1
- Catch2 v2.13 (only for testing)


## License

For non-commercial applications, this software is licensed under the LGPL v3.0. If you want to use frankx within commercial applications or under a different license, please contact us for individual agreements.
