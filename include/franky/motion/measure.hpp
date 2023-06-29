#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

#include "franky/motion/condition.hpp"

#define MEASURE_CMP_DECL(OP) \
Condition operator OP(const Measure &m1, const Measure &m2); \
inline Condition operator OP(const Measure &m1, double m2) { \
  return m1 OP Measure(m2); \
} \
inline Condition operator OP(double m1, const Measure &m2) { \
  return Measure(m1) OP m2; \
}

#define MEASURE_OP_DECL(OP) \
Measure operator OP(const Measure &m1, const Measure &m2); \
inline Measure operator OP(const Measure &m1, double m2){ \
  return m1 OP Measure(m2); \
} \
inline Measure operator OP(double m1, const Measure &m2){ \
  return Measure(m1) OP m2; \
}

namespace franky {

class Measure {
  using MeasureFunc = std::function<double(const franka::RobotState &, double, double)>;

 public:
  explicit Measure(MeasureFunc measure_func, std::string repr = "NULL");

  explicit Measure(double constant);

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

MEASURE_CMP_DECL(==)
MEASURE_CMP_DECL(!=)
MEASURE_CMP_DECL(<=)
MEASURE_CMP_DECL(>=)
MEASURE_CMP_DECL(<)
MEASURE_CMP_DECL(>)

MEASURE_OP_DECL(+)
MEASURE_OP_DECL(-)
MEASURE_OP_DECL(*)
MEASURE_OP_DECL(/)

Measure operator-(const Measure &m);

Measure measure_pow(const Measure &base, const Measure &exponent);
inline Measure measure_pow(double base, const Measure &exponent) {
  return measure_pow(Measure(base), exponent);
}
inline Measure measure_pow(const Measure &base, double exponent) {
  return measure_pow(base, Measure(exponent));
}

}  // namespace franky
