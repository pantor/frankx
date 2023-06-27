<div align="center">
  <img width="340" src="https://raw.githubusercontent.com/timschneider42/franky/master/doc/logo.svg?sanitize=true">
  <h3 align="center">
    High-Level Motion Library for the Franka Emika Robot
  </h3>
</div>
<p align="center">
  <a href="https://github.com/timschneider42/franky/actions">
    <img src="https://github.com/timschneider42/franky/workflows/CI/badge.svg" alt="CI">
  </a>

  <a href="https://github.com/timschneider42/franky/actions">
    <img src="https://github.com/timschneider42/franky/workflows/Publish/badge.svg" alt="Publish">
  </a>

  <a href="https://github.com/timschneider42/franky/issues">
    <img src="https://img.shields.io/github/issues/timschneider42/franky.svg" alt="Issues">
  </a>

  <a href="https://github.com/timschneider42/franky/releases">
    <img src="https://img.shields.io/github/v/release/timschneider42/franky.svg?include_prereleases&sort=semver" alt="Releases">
  </a>

  <a href="https://github.com/timschneider42/franky/blob/master/LICENSE">
    <img src="https://img.shields.io/badge/license-LGPL-green.svg" alt="LGPL">
  </a>
</p>


Franky is a high-level motion library (both C++ and Python) for the Franka Emika robot.
It adds a Python wrapper around [libfranka](https://frankaemika.github.io/docs/libfranka.html), while replacing necessary real-time programming with higher-level motion commands.
As franky focuses on making real-time trajectory generation easy, it allows the robot to react to unforeseen events.


## Differences to frankx
Franky is a fork of [frankx](https://github.com/pantor/frankx), though both codebase and functionality differ substantially from frankx by now.
In particular, franky provides the following new feature/improvements:
* [Motions can be updated asynchronously.](#real-time-motion)
* [Reactions allow for the registration of callbacks instead of just printing to stdout when fired.](#real-time-reactions)
* [The robot state is also available during control.](#robot-state)
* A larger part of the libfranka API is exposed to python (e.g.,`setCollisionBehavior`, `setJoinImpedance`, and `setCartesianImpedance`).
* Cartesian motion generation handles boundaries in Euler angles properly.
* [There is a new joint motion type that supports waypoints.](#motion-types)
* [The signature of `Affine` changed.](#geometry) `Affine` does not handle elbow positions anymore. Instead, a new class `RobotPose` stores both the end-effector pose and optionally the elbow position.
* The `MotionData` class does not exist anymore. Instead, reactions and other settings moved to `Motion`.
* [The `Measure` class allows for arithmetic operations.](#real-time-reactions)

## Installation

To start using franky with Python and libfranka *0.9.0*, just install it via
```bash
pip install franky-panda
```

Franky is based on [libfranka](https://github.com/frankaemika/libfranka), [Eigen](https://eigen.tuxfamily.org) for transformation calculations and [pybind11](https://github.com/pybind/pybind11) for the Python bindings.
Franky uses the [Ruckig](https://ruckig.com) Community Version for Online Trajectory Generation (OTG).
As the Franka is quite sensitive to acceleration discontinuities, it requires constrained jerk for all motions.
After installing the dependencies (the exact versions can be found below), you can build and install franky via

```bash
git clone --recurse-submodules git@github.com:timschneider42/franky.git
cd franky
mkdir -p build
cd build
cmake -DBUILD_TYPE=Release ..
make
make install
```

To use franky, you can also include it as a subproject in your parent CMake via `add_subdirectory(franky)` and then `target_link_libraries(<target> libfranky)`. If you need only the Python module, you can install franky via

```bash
pip install .
```

Make sure that the built library can be found from Python by adapting your Python Path.


#### Using Docker

To use franky within Docker we have supplied a [Dockerfile](docker/Dockerfile) which you currently need to build yourself:

```bash
git clone https://github.com/timschneider42/franky.git
cd franky/
docker build -t franky --build-arg libfranka_version=0.7.0 -f docker/Dockerfile .
```

To use another version of libfranka than the default (v.0.7.0) simply change the build argument. Then, to run the container simply:

```bash
docker run -it --rm --network=host --privileged franky
```

The container requires access to the host machines network *and* elevated user rights to allow the docker user to set RT capabilities of the processes run from within it.


## Tutorial

Franky comes with both a C++ and Python API that differ only regarding real-time capability. We will introduce both languages next to each other. In your C++ project, just include `include <franky/franky.hpp>` and link the library. For Python, just `import franky`. As a first example, only four lines of code are needed for simple robotic motions.

```c++
#include <franky/franky.hpp>
using namespace franky;

// Connect to the robot with the FCI IP address
Robot robot("172.16.0.2");

// Reduce velocity and acceleration of the robot
robot.setDynamicRel(0.05);

// Move the end-effector 20cm in positive x-direction
auto motion = LinearMotion(RobotPose(Affine({0.2, 0.0, 0.0}), 0.0), ReferenceType::Relative);

// Finally move the robot
robot.move(motion);
```

The corresponding program in Python is
```python
from franky import Affine, LinearMotion, Robot, ReferenceType

robot = Robot("172.16.0.2")
robot.set_dynamic_rel(0.05)

motion = LinearMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
robot.move(motion)
```

Furthermore, we will introduce methods for geometric calculations, for moving the robot according to different motion types, how to implement real-time reactions and changing waypoints in real time as well as controlling the gripper.


### Geometry

`franky.Affine` is a python wrapper for [Eigen::Affine3d](https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html). 
It is used for Cartesian poses, frames and transformation. 
franky adds its own constructor, which takes a position and a quaternion as inputs:
```python
import math
from scipy.spatial.transform import Rotation
from franky import Affine

z_translation = Affine([0.0, 0.0, 0.5])

quat = Rotation.from_euler("xyz", [0, 0, math.pi / 2]).as_quat()
z_rotation = Affine([0.0, 0.0, 0.0], quat)

combined_transformation = z_translation * z_rotation
```

In all cases, distances are in [m] and rotations in [rad].

### Robot

We wrapped most of the libfanka API (including the RobotState or ErrorMessage) for Python. 
Moreover, we added methods to adapt the dynamics of the robot for all motions. 
The `rel` name denotes that this a factor of the maximum constraints of the robot.
```python
from franky import Robot

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
current_pose = robot.current_pose
```


### Robot State

The robot state can be retrieved by calling the following methods:

* `state`: Return an object of the `franky.RobotState` class which contains most of the same attributes, under the same name, as the libfranka [franka::RobotState](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html) definition.

* `current_pose`: Return a 3D Affine transformation object of the measured end effector pose in base frame (alias for [franka::RobotState::O_T_EE](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#a193781d47722b32925e0ea7ac415f442)).

* `current_joint_positions`: Return a sequence of the manipulator arm's 7-joint positions (alias for [franka::RobotState::q](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html#ade3335d1ac2f6c44741a916d565f7091)).

```.py
robot = Robot("172.16.0.2")

# Get the current state
state = robot.state
pose = robot.current_pose
joint_positions = robot.current_joint_positions
```


### Motion Types

Franky defines a number of different motion types. 
In python, you can use them as follows:
```python
import math
from scipy.spatial.transform import Rotation
from franky import JointPositionMotion, JointWaypointMotion, JointWaypoint, JointPositionStopMotion, LinearMotion, CartesianWaypointMotion, CartesianWaypoint, Affine, RobotPose, ReferenceType, CartesianPoseStopMotion

# A point-to-point motion in the joint space
m1 = JointPositionMotion([-1.8, 1.1, 1.7, -2.1, -1.1, 1.6, -0.4])

# Generalization of JointPositionMotion that allows for multiple waypoints
m2 = JointWaypointMotion([
    JointWaypoint([-1.8, 1.1, 1.7, -2.1, -1.1, 1.6, -0.4]),
    JointWaypoint([-1.7, 1.2, 1.8, -2.0, -1.0, 1.7, -0.3]),
    JointWaypoint([-1.9, 1.0, 1.6, -2.2, -1.2, 1.5, -0.5])
])

# Stop the robot
m3 = JointPositionStopMotion()

# A linear motion in cartesian space
quat = Rotation.from_euler("xyz", [0, 0, math.pi / 2]).as_quat()
m4 = LinearMotion(Affine([0.2, -0.4, 0.3], quat))
m5 = LinearMotion(RobotPose(Affine([0.2, -0.4, 0.3], quat), elbow_position=1.7)) # With target elbow angle

# A linear motion in cartesian space relative to the initial position
# (Note that this motion is relative both in position and orientation. Hence, when the robot's end-effector is oriented
# differently, it will move in a different direction)
m6 = LinearMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)

# Generalization of LinearMotion that allows for multiple waypoints
m7 = CartesianWaypointMotion([
  CartesianWaypoint(RobotPose(Affine([0.2, -0.4, 0.3], quat), elbow_position=1.7)),
  # The following waypoint is relative to the prior one and 50% slower
  CartesianWaypoint(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative, velocity_rel=0.5)
])

# Stop the robot. The difference of JointPositionStopMotion to CartesianPoseStopMotion is that JointPositionStopMotion
# stops the robot in joint position control mode while CartesianPoseStopMotion stops it in cartesian pose control mode.
# The difference becomes relevant when asynchronous move commands are being sent (see below).
m8 = CartesianPoseStopMotion()
```

Every motion and waypoint type allows to adapt the dynamics (velocity, acceleration and jerk) by setting the respective `velocity_rel`, `acceleration_rel`, and `jerk_rel` parameters.

The real robot can be moved by applying a motion to the robot using `move`:
```python
robot.move(m1)
robot.move(m2)
```


### Real-Time Reactions

By adding reactions to the motion data, the robot can react to unforeseen events. 
In the Python API, you can define conditions by using a comparison between a robot's value and a given threshold. 
If the threshold is exceeded, the reaction fires. 
```python
from franky import LinearMotion, Affine, ReferenceType, Measure, Reaction

motion = LinearMotion(Affine([0.0, 0.0, 0.1]), ReferenceType.Relative)  # Move down 10cm

reaction_motion = LinearMotion(Affine([0.0, 0.0, 0.01]), ReferenceType.Relative)  # Move up for 1cm

# Trigger reaction if the Z force is greater than 30N
reaction = Reaction(Measure.FORCE_Z > 30.0, reaction_motion)
motion.add_reaction(reaction)

robot.move(motion)
```

Possible values to measure are
* `Measure.FORCE_X,` `Measure.FORCE_Y,` `Measure.FORCE_Z`: Force in X, Y and Z direction
* `Measure.REL_TIME`: Time in seconds since the current motion started
* `Measure.ABS_TIME`: Time in seconds since the initial motion started

The difference between `Measure.REL_TIME` and `Measure.ABS_TIME` is that `Measure.REL_TIME` is reset to zero whenever a new motion starts (either by calling `Robot.move` or as a result of a triggered `Reaction`).
`Measure.ABS_TIME`, on the other hand, is only reset to zero when a motion terminates regularly without being interrupted and the robot stops moving.
Hence, `Measure.ABS_TIME` measures the total time in which the robot has moved without interruption.

`Measure` values support all classical arithmetic operations, like addition, subtraction, multiplication, division, and exponentiation (both as base and exponent).
```python
normal_force = (Measure.FORCE_X ** 2 + Measure.FORCE_Y ** 2 + Measure.FORCE_Z ** 2) ** 0.5
```

With arithmetic comparisons, conditions can be generated.
```python
normal_force_within_bounds = normal_force < 30.0
time_up = Measure.ABS_TIME > 10.0
```

Conditions support negation, conjunction (and), and disjunction (or):
```python
abort = not normal_force_within_bounds or time_up
fast_abort = not normal_force_within_bounds and time_up
```

To check whether a reaction has fired, a callback can be attached:
```python
from franky import RobotState

def reaction_callback(robot_state: RobotState, rel_time: float, abs_time: float):
    print(f"Reaction fired at {abs_time}.")

reaction.register_callback(reaction_callback)
```

Note that these callbacks are not executed in the control thread since they would otherwise block it.
Instead, they are put in a queue and executed by another thread.
While this scheme ensures that the control thread can always run, it cannot prevent that the queue grows indefinitely when the callbacks take more time to execute than it takes for new callbacks to be queued.
Hence, callbacks might be executed significantly after their respective reaction has fired if they are triggered in rapid succession or take a long time to execute.

In C++ you can additionally use lambdas to define more complex behaviours:
```c++
auto motion = LinearMotion(RobotPose(Affine({0.0, 0.0, 0.2}), 0.0), ReferenceType::Relative);

// Stop motion if force is over 10N
auto stop_motion = StopMotion<franka::CartesianPose>()

motion
  .addReaction(
    Reaction(
      Measure::ForceZ() > 10.0,  // [N],
      stop_motion))
  .addReaction(
    Reaction(
      Condition(
        [](const franka::RobotState& state, double rel_time, double abs_time) {
          // Lambda condition
          return state.current_errors.self_collision_avoidance_violation;
        }),
      [](const franka::RobotState& state, double rel_time, double abs_time) {
        // Lambda reaction motion generator
        // (we are just returning a stop motion, but there could be arbitrary 
        // logic here for generating reaction motions)
        return StopMotion<franka::CartesianPose>();
      })
    ));

robot.move(motion)
```


### Real-Time Motion

By setting the `asynchronous` parameter of `Robot.move` to `True`, the function does not block until the motion finishes.
Instead, it returns immediately and, thus, allows the main thread to set new motions asynchronously. 
```python
import time
from franky import Affine, LinearMotion, Robot, ReferenceType

robot = Robot("172.16.0.2")
robot.set_dynamic_rel(0.05)

motion1 = LinearMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
robot.move(motion1, asynchronous=True)

time.sleep(0.5)
motion2 = LinearMotion(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative)
robot.move(motion2, asynchronous=True)
```

By calling `Robot.join_motion` the main thread can be synchronized with the motion thread, as it will block until the robot finishes its motion.
```python
robot.join_motion()
```

Note that when exceptions occur during the asynchronous execution of a motion, they will not be thrown immediately.
Instead, the control thread stores the exception and terminates.
The next time `Robot.join_motion` or `Robot.move` are called, they will throw the stored exception in the main thread.
Hence, after an asynchronous motion has finished, make sure to call `Robot.join_motion` to ensure being notified of any exceptions that occurred during the motion.


### Gripper

In the `franky::Gripper` class, the default gripper force and gripper speed can be set. 
Then, additionally to the libfranka commands, the following helper methods can be used:

```c++
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

The Python API is straight-forward for the Gripper class.


### Kinematics

Franky includes a rudimentary, non-realtime-capable forward and inverse kinematics.

```python
from franky import Kinematics, NullSpaceHandling, Affine

# Some initial joint configuration
q = [-1.45549, 1.15401, 1.50061, -2.30909, -1.3141, 1.9391, 0.02815]

# Calculate the forward kinematics
x = Kinematics.forward(q)
print(f"Effector position: {x}")

# Define new target position
x_new = Affine([0.1, 0.0, 0.0]) * x

# Franka has 7 DoFs, so what to do with the remaining Null space?
null_space = NullSpaceHandling(2, 1.4) # Set elbow joint to 1.4

# Inverse kinematic with target, initial joint angles, and Null space configuration
q_new = Kinematics.inverse(x_new.vector(), q, null_space)

print(f"New position: {x_new}")
print(f"New joints: {q_new}")
```


## Documentation

An auto-generated documentation can be found at [https://timschneider42.github.io/franky/](https://timschneider42.github.io/franky/).
Moreover, there are multiple examples for both C++ and Python in the [examples](https://github.com/TimSchneider42/franky/tree/master/examples) directory. 
We will add a more detailed documentation once franky reaches v1.0.


## Development

Franky is written in C++17 and Python3.7. It is currently tested against following versions

- Eigen v3.3.7
- Libfranka v0.9.0
- Pybind11 v2.9.1
- Catch2 v2.13 (only for testing)


## License

For non-commercial applications, this software is licensed under the LGPL v3.0. 
If you want to use franky within commercial applications or under a different license, please contact us for individual agreements.
