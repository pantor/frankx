#include <frankx/gripper.hpp>


namespace frankx {

Gripper::Gripper(std::string fci_ip): franka::Gripper(fci_ip) { }

bool Gripper::move(double width) {
  return ((franka::Gripper*) this)->move(width, gripper_speed); // [m] [m/s]
}

bool Gripper::clamp() {
  return this->grasp(0.0, gripper_speed, gripper_force, 0.0, 0.08); // [m] [m/s] [N] [m] [m]
}

bool Gripper::release(double width) {
  this->stop();
  return this->move(width);
}

} // namepace frankx
