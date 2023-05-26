#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

#include "franky/motion/condition.hpp"

#define MEASURE_OP_DECL(OP) \
Condition operator OP(const Measure &m1, const Measure &m2); \
Condition operator OP(const Measure &m1, double m2); \
Condition operator OP(double m1, const Measure &m2);

namespace franky {

class Measure {
  using MeasureFunc = std::function<double(const franka::RobotState &, double)>;

 public:
  explicit Measure(MeasureFunc measure_func, std::string repr = "NULL");

  explicit Measure(double constant);

  inline double operator()(const franka::RobotState &robot_state, double time) const {
    return measure_func_(robot_state, time);
  }

  static Measure ForceX();

  static Measure ForceY();

  static Measure ForceZ();

  static Measure Time();

  std::string repr() const {
    return repr_;
  }

 private:
  MeasureFunc measure_func_;
  std::string repr_;
};

Condition operator&&(const Condition &c1, const Condition &c2);

Condition operator||(const Condition &c1, const Condition &c2);

Condition operator!(const Condition &c);

MEASURE_OP_DECL(==)
MEASURE_OP_DECL(!=)
MEASURE_OP_DECL(<=)
MEASURE_OP_DECL(>=)
MEASURE_OP_DECL(<)
MEASURE_OP_DECL(>)

}  // namespace franky
