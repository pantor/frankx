#include "franky/motion/measure.hpp"

#include <cmath>
#include <utility>
#include <franka/robot_state.h>

namespace franky {
  Measure::Measure(Measure::MeasureFunc measure_func) : measure_func_(std::move(measure_func)) {}

  Measure::Measure(double constant)
      : measure_func_([constant](const franka::RobotState &robot_state, double time) {
    return constant;
  }) {}

  Measure Measure::ForceX() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return robot_state.O_F_ext_hat_K[0];
    });
  }

  Measure Measure::ForceY() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return robot_state.O_F_ext_hat_K[1];
    });
  }

  Measure Measure::ForceZ() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return robot_state.O_F_ext_hat_K[2];
    });
  }

  Measure Measure::ForceXYNorm() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
    });
  }

  Measure Measure::ForceXYZNorm() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) +
             std::pow(robot_state.O_F_ext_hat_K[2], 2);
    });
  }

  Measure Measure::Time() {
    return Measure([](const franka::RobotState &robot_state, double time) {
      return time;
    });
  }
} // namespace franky
