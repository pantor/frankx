#pragma once

#include <franka/exception.h>
#include <franka/gripper.h>


namespace frankx {

class Gripper: public franka::Gripper {
public:
  /**
   * Connects to a gripper at the given FCI IP address.
   */
  Gripper(std::string fci_ip);


  double gripper_force {20.0}; // [N]
  double gripper_speed {0.03}; // [m/s]

  bool move(double width); // [m]

  bool clamp();

  bool release(double width); // [m]
};

} // namepace frankx
