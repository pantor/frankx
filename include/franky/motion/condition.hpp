#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

namespace franky {

class Condition {
 public:
  using CheckFunc = std::function<bool(const franka::RobotState &, double, double)>;

  explicit Condition(CheckFunc check_func, std::string repr = "NULL");

  Condition(bool constant_value);

  //! Check if the condition is fulfilled
  inline bool operator()(const franka::RobotState &robot_state, double rel_time, double abs_time) const {
    return check_func_(robot_state, rel_time, abs_time);
  }

  [[nodiscard]] inline std::string repr() const {
    return repr_;
  }

 private:
  CheckFunc check_func_;
  std::string repr_;
};

Condition operator&&(const Condition &c1, const Condition &c2);

Condition operator||(const Condition &c1, const Condition &c2);

Condition operator==(const Condition &c1, const Condition &c2);

Condition operator!=(const Condition &c1, const Condition &c2);

Condition operator!(const Condition &c);

}  // namespace franky
