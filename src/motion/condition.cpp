#include "franky/motion/condition.hpp"

#include <franka/robot_state.h>

#include <utility>

namespace franky {

Condition::Condition(Condition::CheckFunc check_func) : check_func_(std::move(check_func)) {}

Condition operator&&(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) && c2(robot_state, time);
  });
}

Condition operator||(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) || c2(robot_state, time);
  });
}

Condition operator!(const Condition &c) {
  return Condition([c](const franka::RobotState &robot_state, double time) {
    return !c(robot_state, time);
  });
}

}  // namespace franky
