#include "franky/gripper.hpp"

namespace franky {

Gripper::Gripper(const std::string &fci_hostname, double speed, double force)
    : franka::Gripper(fci_hostname), gripper_speed(speed), gripper_force(force) {}

double Gripper::width() const {
  auto state = readOnce();
  return state.width + width_calibration;
}

bool Gripper::isGrasping() const {
  const double current_width = width();
  const bool libfranka_is_grasped = readOnce().is_grasped;
  const bool width_is_grasped = std::abs(current_width - last_clamp_width) < 0.003; // [m], magic number
  const bool width_larger_than_threshold = current_width > 0.005; // [m]
  return libfranka_is_grasped && width_is_grasped && width_larger_than_threshold;
}

bool Gripper::move(double width) { // [m]
  try {
    const bool result = ((franka::Gripper *) this)->move(width - width_calibration, gripper_speed); // [m] [m/s]
    const double current_width = this->width();
    if (current_width > 0.01 && std::abs(current_width - width) > 0.01) {
      has_error = true;
      throw GripperException(
          "Gripper does (" + std::to_string(current_width) + ") not do what it should (" + std::to_string(width) +
              ").");
      return false;
    }
    if (result) {
      has_error = false;
    }
    return result;
  } catch (franka::Exception const &e) {
    has_error = true;
    std::cout << e.what() << std::endl;
    this->stop();
    this->homing();
    return ((franka::Gripper *) this)->move(width - width_calibration, gripper_speed); // [m] [m/s]
  }
}

bool Gripper::moveUnsafe(double width) { // [m]
  const bool result_stop = ((franka::Gripper *) this)->stop();
  return ((franka::Gripper *) this)->move(width - width_calibration, gripper_speed); // [m] [m/s]
}

std::future<bool> Gripper::moveAsync(double width) { // [m]
  return std::async(std::launch::async, &Gripper::move, this, width - width_calibration);
}

bool Gripper::open() {
  return move(max_width);
}

bool Gripper::clamp() {
  const bool success = grasp(min_width, gripper_speed, gripper_force, min_width, 1.0); // [m] [m/s] [N] [m] [m]
  last_clamp_width = width();
  return success;
}

bool Gripper::clamp(double min_clamping_width) {
  const bool success = this->grasp(min_clamping_width, gripper_speed, gripper_force, min_width,
                                   1.0); // [m] [m/s] [N] [m] [m]
  last_clamp_width = this->width();
  return success;
}

bool Gripper::release() { // [m]
  return release(last_clamp_width);
}

bool Gripper::release(double width) { // [m]
  try {
    // stop();
    return move(width);
  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    homing();
    stop();
    return move(width);
  }
}

bool Gripper::releaseRelative(double width_relative) { // [m]
  return release(width() + width_relative);
}

::franka::GripperState Gripper::get_state() {
  return readOnce();
}

} // namepace franky
