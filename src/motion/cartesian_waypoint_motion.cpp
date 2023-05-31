#include "franky/motion/cartesian_waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>
#include <utility>

#include "franky/robot_pose.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"

namespace franky {

CartesianWaypointMotion::CartesianWaypointMotion(const std::vector<CartesianWaypoint> &waypoints)
    : CartesianWaypointMotion(waypoints, Params()) {}

CartesianWaypointMotion::CartesianWaypointMotion(const std::vector<CartesianWaypoint> &waypoints, Params params)
    : params_(std::move(params)), WaypointMotion<franka::CartesianPose, CartesianWaypoint>(waypoints, params) {}

void CartesianWaypointMotion::initWaypointMotion(
    const franka::RobotState &robot_state, ruckig::InputParameter<7> &input_parameter) {
  franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
  RobotPose robot_pose(initial_cartesian_pose);
  ref_frame_ = Affine::Identity();

  auto initial_velocity = Vector<6>::Map(robot_state.O_dP_EE_c.data());
  Vector7d initial_velocity_with_elbow = (Vector7d() << initial_velocity, robot_state.delbow_c[0]).finished();

  auto initial_acceleration = Vector<6>::Map(robot_state.O_ddP_EE_c.data());
  Vector7d initial_acceleration_with_elbow = (Vector7d() << initial_acceleration, robot_state.ddelbow_c[0]).finished();

  target_robot_pose_ = robot_pose;

  input_parameter.current_position = toStd<7>(robot_pose.vector_repr());
  input_parameter.current_velocity = toStd<7>(initial_velocity_with_elbow);
  input_parameter.current_acceleration = toStd<7>(initial_acceleration_with_elbow);
}

franka::CartesianPose CartesianWaypointMotion::getControlSignal(
    const ruckig::InputParameter<7> &input_parameter) const {
  auto has_elbow = input_parameter.enabled[6];
  return (ref_frame_ * RobotPose(toEigen<7>(input_parameter.current_position), !has_elbow)).as_franka_pose();
}

void CartesianWaypointMotion::setNewWaypoint(
    const franka::RobotState &robot_state,
    const CartesianWaypoint &new_waypoint,
    ruckig::InputParameter<7> &input_parameter) {
  auto waypoint_has_elbow = input_parameter.enabled[6];

  // We first convert the current state into the frame of the current pose
  RobotPose current_pose_old_ref_frame = RobotPose(toEigen<7>(input_parameter.current_position), !waypoint_has_elbow);
  Affine new_ref_to_old_ref = current_pose_old_ref_frame.end_effector_pose();
  ref_frame_ = ref_frame_ * new_ref_to_old_ref;
  auto rot = new_ref_to_old_ref.inverse().rotation();

  Vector<7> current_velocity = toEigen<7>(input_parameter.current_velocity);
  auto linear_vel_ref_frame = rot * current_velocity.head<3>();
  auto angular_vel_ref_frame = rot * current_velocity.segment<3>(3);

  Vector<7> current_acc = toEigen<7>(input_parameter.current_acceleration);
  auto linear_acc_ref_frame = rot * current_acc.head<3>();
  auto angular_acc_ref_frame = rot * current_acc.segment<3>(3);

  double elbow_velocity, elbow_acc;
  if (waypoint_has_elbow) {
    elbow_velocity = current_velocity[6];
    elbow_acc = current_acc[6];
  } else {
    elbow_velocity = robot_state.delbow_c[6];
    elbow_acc = robot_state.ddelbow_c[6];
  }

  RobotPose zero_pose(Affine::Identity(), current_pose_old_ref_frame.elbow_position());
  Vector7d current_velocity_ref_frame =
      (Vector7d() << linear_vel_ref_frame, angular_vel_ref_frame, elbow_velocity).finished();
  Vector7d current_acc_ref_frame =
      (Vector7d() << linear_acc_ref_frame, angular_acc_ref_frame, elbow_acc).finished();
  input_parameter.current_position = toStd<7>(zero_pose.vector_repr());
  input_parameter.current_velocity = toStd<7>(current_velocity_ref_frame);
  input_parameter.current_acceleration = toStd<7>(current_acc_ref_frame);

  waypoint_has_elbow = new_waypoint.robot_pose.elbow_position().has_value();

  auto prev_target_robot_pose = target_robot_pose_;
  if (!target_robot_pose_.elbow_position().has_value()) {
    prev_target_robot_pose = prev_target_robot_pose.with_elbow_position(robot_state.elbow[0]);
  }

  std::optional<double> new_elbow;
  if (new_waypoint.robot_pose.elbow_position().has_value() && new_waypoint.reference_type == ReferenceType::Relative) {
    if (!prev_target_robot_pose.elbow_position().has_value())
      new_elbow = new_waypoint.robot_pose.elbow_position();
    else {
      new_elbow = new_waypoint.robot_pose.elbow_position().value() + prev_target_robot_pose.elbow_position().value();
    }
  } else {
    new_elbow = new_waypoint.robot_pose.elbow_position();
  }
  RobotPose new_target_robot_pose = new_waypoint.robot_pose.with_elbow_position(new_elbow);
  if (new_waypoint.reference_type == ReferenceType::Relative)
    new_target_robot_pose = prev_target_robot_pose.end_effector_pose() * new_target_robot_pose;

  auto new_target_robot_pose_transformed = new_target_robot_pose * params_.frame.inverse();

  auto new_target_robot_pose_ref_frame = ref_frame_.inverse() * new_target_robot_pose_transformed;
  input_parameter.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
  input_parameter.target_position = toStd<7>(new_target_robot_pose_ref_frame.vector_repr());
  input_parameter.target_velocity = toStd<7>(Vector7d::Zero());
  target_robot_pose_ = new_target_robot_pose;
}

std::tuple<Vector7d, Vector7d, Vector7d>
CartesianWaypointMotion::getInputLimits(const CartesianWaypoint &waypoint) const {
  constexpr double translation_factor{0.4};
  constexpr double derivative_factor{0.4};
  const Robot *robot = this->robot();

  Vector7d max_vel = vec_cart_rot_elbow(
      translation_factor * Robot::max_translation_velocity,
      Robot::max_rotation_velocity,
      Robot::max_elbow_velocity);
  Vector7d max_acc = derivative_factor * vec_cart_rot_elbow(
      translation_factor * Robot::max_elbow_acceleration,
      Robot::max_elbow_acceleration,
      Robot::max_elbow_acceleration);
  Vector7d max_jerk = std::pow(derivative_factor, 2) * vec_cart_rot_elbow(
      translation_factor * Robot::max_translation_jerk,
      Robot::max_rotation_jerk,
      Robot::max_elbow_jerk);

  double vel_factor, acc_factor, jerk_factor;

  if (waypoint.max_dynamics || params_.max_dynamics) {
    vel_factor = 1.0;
    acc_factor = 1.0;
    jerk_factor = 1.0;
  } else {
    vel_factor = waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel();
    acc_factor = waypoint.acceleration_rel * params_.acceleration_rel * robot->acceleration_rel();
    jerk_factor = waypoint.jerk_rel * params_.jerk_rel * robot->jerk_rel();
  }

  auto vel_lim = vel_factor * max_vel;
  auto acc_lim = acc_factor * max_acc;
  auto jerk_lim = jerk_factor * max_jerk;
  return {vel_lim, acc_lim, jerk_lim};
}

}  // namespace franky