#include "franky/robot.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"

namespace franky {

using Affine = Eigen::Affine3d;

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_ip) : Robot(fci_ip, Params()) {}

Robot::Robot(const std::string &fci_ip, const Params &params)
    : fci_ip_(fci_ip), params_(params), franka::Robot(fci_ip, params.realtime_config) {
  setCollisionBehavior(params_.default_torque_threshold, params_.default_force_threshold);
}

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
  return {Affine(Eigen::Matrix4d::Map(s.O_T_EE.data())), s.elbow[0]};
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
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  {
    std::lock_guard<std::mutex> control_lock(control_mutex_);
    if (!is_in_control_unsafe()) {
      current_state_ = readOnce();
    }
  }
  return current_state_;
}

void Robot::setCollisionBehavior(double torque_threshold, double force_threshold) {
  setCollisionBehavior(mkFull<7>(torque_threshold), mkFull<6>(force_threshold));
}

void Robot::setCollisionBehavior(const std::array<double, 7> &torque_thresholds,
                                 const std::array<double, 6> &force_thresholds) {
  setCollisionBehavior(torque_thresholds, torque_thresholds, force_thresholds, force_thresholds);
}

void Robot::setCollisionBehavior(
    double lower_torque_threshold,
    double upper_torque_threshold,
    double lower_force_threshold,
    double upper_force_threshold) {
  setCollisionBehavior(
      mkFull<7>(lower_torque_threshold),
      mkFull<7>(upper_torque_threshold),
      mkFull<6>(lower_force_threshold),
      mkFull<6>(upper_force_threshold));
}

void Robot::setCollisionBehavior(
    double lower_torque_threshold_acceleration,
    double upper_torque_threshold_acceleration,
    double lower_torque_threshold_nominal,
    double upper_torque_threshold_nominal,
    double lower_force_threshold_acceleration,
    double upper_force_threshold_acceleration,
    double lower_force_threshold_nominal,
    double upper_force_threshold_nominal) {
  setCollisionBehavior(
      mkFull<7>(lower_torque_threshold_acceleration),
      mkFull<7>(upper_torque_threshold_acceleration),
      mkFull<7>(lower_torque_threshold_nominal),
      mkFull<7>(upper_torque_threshold_nominal),
      mkFull<6>(lower_force_threshold_acceleration),
      mkFull<6>(upper_force_threshold_acceleration),
      mkFull<6>(lower_force_threshold_nominal),
      mkFull<6>(upper_force_threshold_nominal));

}

bool Robot::is_in_control_unsafe() const {
  return control_future_.has_value()
      && control_future_->wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}

bool Robot::is_in_control() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return is_in_control_unsafe();
}

void Robot::joinMotion() {
  // This is to ensure the control thread safety of this operation. Otherwise, it is possible that the control_thread
  // pointer gets overwritten at the exact moment we try to join the control_thread.
  std::optional<std::shared_future<std::exception_ptr>> control_future;
  {
    std::unique_lock<std::mutex> lock(control_mutex_);
    control_future = control_future_;
  }
  if (control_future.has_value())
    control_future->wait();
  auto control_exception = control_future->get();
  if (control_exception != nullptr)
    std::rethrow_exception(control_exception);
}

}  // namespace franky
