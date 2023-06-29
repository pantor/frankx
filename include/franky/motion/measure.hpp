#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

#include "franky/motion/condition.hpp"

namespace franky {

class Measure {
  using MeasureFunc = std::function<double(const franka::RobotState &, double, double)>;

 public:
  explicit Measure(MeasureFunc measure_func, std::string repr = "NULL");

  Measure(double constant);

  inline double operator()(const franka::RobotState &robot_state, double rel_time, double abs_time) const {
    return measure_func_(robot_state, rel_time, abs_time);
  }

  [[nodiscard]] inline std::string repr() const {
    return repr_;
  }

  static Measure ForceX();

  static Measure ForceY();

  static Measure ForceZ();

  static Measure RelTime();

  static Measure AbsTime();

 private:
  MeasureFunc measure_func_;
  std::string repr_;
};

Condition operator==(const Measure &m1, const Measure &m2);
Condition operator!=(const Measure &m1, const Measure &m2);
Condition operator<=(const Measure &m1, const Measure &m2);
Condition operator>=(const Measure &m1, const Measure &m2);
Condition operator<(const Measure &m1, const Measure &m2);
Condition operator>(const Measure &m1, const Measure &m2);

Measure operator+(const Measure &m1, const Measure &m2);
Measure operator-(const Measure &m1, const Measure &m2);
Measure operator*(const Measure &m1, const Measure &m2);
Measure operator/(const Measure &m1, const Measure &m2);

Measure operator-(const Measure &m);

Measure measure_pow(const Measure &base, const Measure &exponent);

}  // namespace franky
