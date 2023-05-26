#include "franky/motion/condition.hpp"

#include <utility>
#include <sstream>
#include <franka/robot_state.h>

namespace franky {

Condition::Condition(Condition::CheckFunc check_func, std::string name)
    : check_func_(std::move(check_func)), name_(std::move(name)) {}

Condition operator&&(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) && c2(robot_state, time);
  }, "(" + c1.name() + ") && (" + c2.name() + ")");
}

Condition operator||(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) || c2(robot_state, time);
  }, c1.name() + " || " + c2.name());
}

Condition operator!(const Condition &c) {
  return Condition([c](const franka::RobotState &robot_state, double time) {
    return !c(robot_state, time);
  }, "!(" + c.name() + ")");
}

}  // namespace franky
