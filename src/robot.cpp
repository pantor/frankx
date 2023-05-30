#include <ruckig/ruckig.hpp>

#include "franky/robot.hpp"

namespace franky {

using Affine = Eigen::Affine3d;

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_ip) : Robot(fci_ip, Params()) {}

Robot::Robot(const std::string &fci_ip, const Params &params)
    : fci_ip_(fci_ip), params_(params), is_in_control_(false), franka::Robot(fci_ip, params.realtime_config) {}

void Robot::setDynamicRel(double dynamic_rel) {
  setDynamicRel(dynamic_rel, dynamic_rel, dynamic_rel);
}

void Robot::setDynamicRel(double velocity_rel, double acceleration_rel, double jerk_rel) {
  params_.velocity_rel = velocity_rel;
  params_.acceleration_rel = acceleration_rel;
  params_.jerk_rel = jerk_rel;
}

bool Robot::hasErrors() {
  return bool(state().current_errors);
}

bool Robot::recoverFromErrors() {
  automaticErrorRecovery();
  return !hasErrors();
}

RobotPose Robot::currentPose() {
  auto s = state();
  return RobotPose(Affine(Eigen::Matrix4d::Map(s.O_T_EE.data())), s.elbow[0]);
}

Vector7d Robot::currentJointPositions() {
  return Eigen::Map<const Vector7d>(state().q.data());
}

Affine Robot::forwardKinematics(const Vector7d &q) {
  return Kinematics::forward(q);
}

Vector7d Robot::inverseKinematics(const Affine &target, const Vector7d &q0) {
  Vector7d result;

  Eigen::Vector3d angles = Euler(target.rotation()).angles();
  Eigen::Vector3d angles_norm;
  angles_norm << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;

  if (angles_norm[1] > M_PI) {
    angles_norm[1] -= 2 * M_PI;
  }
  if (angles_norm[2] < -M_PI) {
    angles_norm[2] += 2 * M_PI;
  }

  if (angles.norm() < angles_norm.norm()) {
    angles_norm = angles;
  }

  Eigen::Matrix<double, 6, 1> x_target;
  x_target << target.translation(), angles;

  return Kinematics::inverse(x_target, q0);
}

franka::RobotState Robot::state() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!is_in_control_) {
    current_state_ = readOnce();
  }
  return current_state_;
}

}  // namespace franky
