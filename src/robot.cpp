#include "franky/robot.hpp"

#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/types.hpp"

namespace franky {

//! Connects to a robot at the given FCI IP address.
Robot::Robot(const std::string &fci_hostname) : Robot(fci_hostname, Params()) {}

Robot::Robot(const std::string &fci_hostname, const Params &params)
    : fci_hostname_(fci_hostname), params_(params), franka::Robot(fci_hostname, params.realtime_config) {
  setCollisionBehavior(params_.default_torque_threshold, params_.default_force_threshold);
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

void Robot::setCollisionBehavior(const ScalarOrArray<7> &torque_threshold, const ScalarOrArray<6> &force_threshold) {
  setCollisionBehavior(torque_threshold, torque_threshold, force_threshold, force_threshold);
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold,
    const ScalarOrArray<7> &upper_torque_threshold,
    const ScalarOrArray<6> &lower_force_threshold,
    const ScalarOrArray<6> &upper_force_threshold) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold),
      expand<7>(upper_torque_threshold),
      expand<6>(lower_force_threshold),
      expand<6>(upper_force_threshold));
}

void Robot::setCollisionBehavior(
    const ScalarOrArray<7> &lower_torque_threshold_acceleration,
    const ScalarOrArray<7> &upper_torque_threshold_acceleration,
    const ScalarOrArray<7> &lower_torque_threshold_nominal,
    const ScalarOrArray<7> &upper_torque_threshold_nominal,
    const ScalarOrArray<6> &lower_force_threshold_acceleration,
    const ScalarOrArray<6> &upper_force_threshold_acceleration,
    const ScalarOrArray<6> &lower_force_threshold_nominal,
    const ScalarOrArray<6> &upper_force_threshold_nominal) {
  franka::Robot::setCollisionBehavior(
      expand<7>(lower_torque_threshold_acceleration),
      expand<7>(upper_torque_threshold_acceleration),
      expand<7>(lower_torque_threshold_nominal),
      expand<7>(upper_torque_threshold_nominal),
      expand<6>(lower_force_threshold_acceleration),
      expand<6>(upper_force_threshold_acceleration),
      expand<6>(lower_force_threshold_nominal),
      expand<6>(upper_force_threshold_nominal));

}

bool Robot::is_in_control_unsafe() const {
  return motion_generator_running_;
}

bool Robot::is_in_control() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return is_in_control_unsafe();
}

std::string Robot::fci_hostname() const {
  return fci_hostname_;
}

void Robot::joinMotion() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  joinMotionUnsafe(lock);
}

void Robot::joinMotionUnsafe(std::unique_lock<std::mutex> &lock) {
  if (motion_generator_running_)
    control_finished_condition_.wait(lock);
  if (control_thread_.joinable())
    control_thread_.join();
  if (control_exception_ != nullptr) {
    auto control_exception = control_exception_;
    control_exception_ = nullptr;
    std::rethrow_exception(control_exception);
  }
}

std::optional<ControlSignalType> Robot::current_control_signal_type() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  if (!is_in_control())
    return std::nullopt;
  if (std::holds_alternative<MotionGenerator<franka::Torques>>(motion_generator_))
    return ControlSignalType::Torques;
  else if (std::holds_alternative<MotionGenerator<franka::JointVelocities>>(motion_generator_))
    return ControlSignalType::JointVelocities;
  else if (std::holds_alternative<MotionGenerator<franka::JointPositions>>(motion_generator_))
    return ControlSignalType::JointPositions;
  else if (std::holds_alternative<MotionGenerator<franka::CartesianVelocities>>(motion_generator_))
    return ControlSignalType::CartesianVelocities;
  else
    return ControlSignalType::CartesianPose;
}

RelativeDynamicsFactor Robot::relative_dynamics_factor() {
  std::unique_lock<std::mutex> lock(control_mutex_);
  return params_.relative_dynamics_factor;
}

void Robot::setRelativeDynamicsFactor(const RelativeDynamicsFactor &relative_dynamics_factor) {
  std::unique_lock<std::mutex> lock(control_mutex_);
  params_.relative_dynamics_factor = relative_dynamics_factor;
}

}  // namespace franky
