#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

namespace franky {

class Condition {
 public:
  using CheckFunc = std::function<bool(const franka::RobotState &, double)>;

  explicit Condition(CheckFunc callback);

  //! Check if the condition is fulfilled
  inline bool operator()(const franka::RobotState &robot_state, double time) const {
    return check_func_(robot_state, time);
  }

 private:
  CheckFunc check_func_;
};

Condition operator&&(const Condition &c1, const Condition &c2);

Condition operator||(const Condition &c1, const Condition &c2);

Condition operator!(const Condition &c);

}  // namespace franky
