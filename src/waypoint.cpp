#include "franky/waypoint.hpp"
#include "franky/robot_pose.hpp"

namespace franky {

RobotPose Waypoint::getTargetRobotPose(const RobotPose &old_robot_pose) const {
  std::optional<double> new_elbow;
  if (robot_pose.elbow_position().has_value() && reference_type == ReferenceType::Relative) {
    if (!old_robot_pose.elbow_position().has_value())
      new_elbow = robot_pose.elbow_position();
    else {
      new_elbow = robot_pose.elbow_position().value() + old_robot_pose.elbow_position().value();
    }
  } else {
    new_elbow = robot_pose.elbow_position();
  }
  RobotPose new_robot_pose = robot_pose.with_elbow_position(new_elbow);
  if (reference_type == ReferenceType::Absolute)
    return new_robot_pose;
  return old_robot_pose.end_effector_pose() * new_robot_pose;
}

}  // namespace franky