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
      {{{.max_dynamics = true}, Vector7d::Zero(), ReferenceType::Relative}}) {}
};

template<>
class StopMotion<franka::CartesianPose> : public CartesianWaypointMotion {
 public:
  explicit StopMotion() : CartesianWaypointMotion(
      {{{.max_dynamics = true}, RobotPose(Affine::Identity()), ReferenceType::Relative}}) {}
};

}  // namespace franky