#include "franky/motion/condition.hpp"

#include <utility>
#include <sstream>
#include <franka/robot_state.h>

namespace franky {

Condition::Condition(Condition::CheckFunc check_func, std::string repr)
    : check_func_(std::move(check_func)), repr_(std::move(repr)) {}

Condition operator&&(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) && c2(robot_state, time);
  }, "(" + c1.repr() + ") && (" + c2.repr() + ")");
}

Condition operator||(const Condition &c1, const Condition &c2) {
  return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
    return c1(robot_state, time) || c2(robot_state, time);
  }, c1.repr() + " || " + c2.repr());
}

Condition operator!(const Condition &c) {
  return Condition([c](const franka::RobotState &robot_state, double time) {
    return !c(robot_state, time);
  }, "!(" + c.repr() + ")");
}

}  // namespace franky
