#include "franky/motion/measure.hpp"

#include <utility>
#include <franka/robot_state.h>

#define MEASURE_OP_DEF(OP) \
Condition operator OP(const Measure &m1, const Measure &m2) { \
  return Condition([m1, m2](const franka::RobotState &robot_state, double time) { \
    return m1(robot_state, time) OP m2(robot_state, time); \
  }, m1.repr() + " OP " + m2.repr()); \
} \
Condition operator OP(const Measure &m1, double m2) { \
  return m1 OP Measure(m2); \
} \
Condition operator OP(double m1, const Measure &m2) { \
  return Measure(m1) OP m2; \
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

MEASURE_OP_DEF(==)
MEASURE_OP_DEF(!=)
MEASURE_OP_DEF(<=)
MEASURE_OP_DEF(>=)
MEASURE_OP_DEF(<)
MEASURE_OP_DEF(>)

}  // namespace franky
