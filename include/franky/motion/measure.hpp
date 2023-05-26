#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

#include "franky/motion/condition.hpp"

namespace franky {

class Measure {
  using MeasureFunc = std::function<double(const franka::RobotState &, double)>;

 public:
  explicit Measure(MeasureFunc measure_func);

  Measure(double constant);

  inline double operator()(const franka::RobotState &robot_state, double time) const {
    return measure_func_(robot_state, time);
  }

  static Measure ForceX();

  static Measure ForceY();

  static Measure ForceZ();

  static Measure Time();

 private:
  MeasureFunc measure_func_;
};

Condition operator==(const Measure &m1, const Measure &m2);

Condition operator!=(const Measure &m1, const Measure &m2);

Condition operator<=(const Measure &m1, const Measure &m2);

Condition operator>=(const Measure &m1, const Measure &m2);

Condition operator<(const Measure &m1, const Measure &m2);

Condition operator>(const Measure &m1, const Measure &m2);

}  // namespace franky
