#include "franky/motion/measure.hpp"

#include <sstream>
#include <utility>
#include <franka/robot_state.h>

#define MEASURE_CMP_DEF(OP) \
Condition operator OP(const Measure &m1, const Measure &m2) { \
  std::stringstream ss;     \
  ss << m1.repr() << " " << #OP << " " << m2.repr(); \
  return Condition([m1, m2](const franka::RobotState &robot_state, double time) { \
    return m1(robot_state, time) OP m2(robot_state, time); \
  }, ss.str()); \
}

#define MEASURE_OP_DEF(OP) \
Measure operator OP(const Measure &m1, const Measure &m2) { \
  std::stringstream ss;     \
  ss << "(" << m1.repr() << ") " << #OP << " (" << m2.repr() << ")"; \
  return Measure([m1, m2](const franka::RobotState &robot_state, double time) { \
    return m1(robot_state, time) OP m2(robot_state, time); \
  }, ss.str()); \
}

namespace franky {

Measure::Measure(Measure::MeasureFunc measure_func, std::string repr)
    : measure_func_(std::move(measure_func)), repr_(std::move(repr)) {}

Measure::Measure(double constant)
    : measure_func_([constant](const franka::RobotState &robot_state, double time) {
  return constant;
}), repr_(std::to_string(constant)) {}

Measure Measure::ForceX() {
  return Measure([](const franka::RobotState &robot_state, double time) {
    return robot_state.O_F_ext_hat_K[0];
  }, "F_x");
}

Measure Measure::ForceY() {
  return Measure([](const franka::RobotState &robot_state, double time) {
    return robot_state.O_F_ext_hat_K[1];
  }, "F_y");
}

Measure Measure::ForceZ() {
  return Measure([](const franka::RobotState &robot_state, double time) {
    return robot_state.O_F_ext_hat_K[2];
  }, "F_z");
}

Measure Measure::Time() {
  return Measure([](const franka::RobotState &robot_state, double time) {
    return time;
  }, "t");
}

MEASURE_CMP_DEF(==)
MEASURE_CMP_DEF(!=)
MEASURE_CMP_DEF(<=)
MEASURE_CMP_DEF(>=)
MEASURE_CMP_DEF(<)
MEASURE_CMP_DEF(>)

MEASURE_OP_DEF(+)
MEASURE_OP_DEF(-)
MEASURE_OP_DEF(*)
MEASURE_OP_DEF(/)

Measure measure_pow(const Measure &base, const Measure &exponent) {
  std::stringstream ss;
  ss << "(" << base.repr() << ")^(" << exponent.repr() << ")";
  return Measure([base, exponent](const franka::RobotState &robot_state, double time) {
    return std::pow(base(robot_state, time), exponent(robot_state, time));
  }, ss.str());
}

}  // namespace franky
