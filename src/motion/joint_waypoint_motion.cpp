#include "franky/motion/joint_waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/robot_pose.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"

namespace franky {

JointWaypointMotion::JointWaypointMotion(const std::vector<Waypoint<Vector7d>> &waypoints)
    : JointWaypointMotion(waypoints, Params()) {}

JointWaypointMotion::JointWaypointMotion(const std::vector<Waypoint<Vector7d>> &waypoints, Params params)
    : WaypointMotion<franka::JointPositions, Vector7d>(waypoints, params) {}

void JointWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointPositions> &previous_command,
    ruckig::InputParameter<7> &input_parameter) {
  input_parameter.current_position = previous_command->q;
  input_parameter.current_velocity = robot_state.dq_d;
  input_parameter.current_acceleration = robot_state.ddq_d;
}

franka::JointPositions JointWaypointMotion::getControlSignal(
    const ruckig::InputParameter<7> &input_parameter) const {
  return {input_parameter.current_position};
}

void JointWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const std::optional<franka::JointPositions> &previous_command,
    const Waypoint<Vector7d> &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto new_target = new_waypoint.target;
  if (new_waypoint.reference_type == ReferenceType::Relative)
    new_target += toEigen(input_parameter.current_position);
  input_parameter.target_position = toStd<7>(new_target);
  input_parameter.target_velocity = toStd<7>(Vector7d::Zero());
  input_parameter.target_acceleration = toStd<7>(Vector7d::Zero());
}

std::tuple<Vector7d, Vector7d, Vector7d> JointWaypointMotion::getAbsoluteInputLimits() const {
  return {Vector7d::Map(Robot::max_joint_velocity.data()),
          (Vector7d::Map(Robot::max_joint_acceleration.data()) * 0.3).eval(),
          (Vector7d::Map(Robot::max_joint_jerk.data()) * 0.3).eval()};
}
}  // namespace franky
