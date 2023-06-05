#pragma once

#include "franky/motion/reference_type.hpp"
#include "franky/motion/joint_waypoint_motion.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"

namespace franky {

template<typename ControlSignalType>
class StopMotion;

template<>
class StopMotion<franka::JointPositions> : public JointWaypointMotion {
 public:
  explicit StopMotion() : JointWaypointMotion(
      {{.target=Vector7d::Zero(), .reference_type= ReferenceType::Relative, .max_dynamics = true}}) {}
};

template<>
class StopMotion<franka::CartesianPose> : public CartesianWaypointMotion {
 public:
  explicit StopMotion() : CartesianWaypointMotion(
      {{.target = RobotPose(Affine::Identity()), .reference_type = ReferenceType::Relative, .max_dynamics = true}}) {}
};

}  // namespace franky