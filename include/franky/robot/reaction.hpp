#pragma once

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>
#include <franka/robot_state.h>

#include "franky/robot/measure.hpp"
#include "franky/motion/motion.hpp"


namespace franky {

  class WaypointMotion;

  class Reaction {
    using MotionFunc = std::function<std::optional<Motion>(const franka::RobotState &, double)>;

  public:
    explicit Reaction(const Condition &condition, const std::optional<Motion> &new_motion = std::nullopt)
        : Reaction(condition, [new_motion](const franka::RobotState &, double) { return new_motion; }) {}

    explicit Reaction(const Condition &condition, const MotionFunc &motion_func)
        : condition_(condition), motion_func_(motion_func) {}

    std::optional<Motion> operator()(const franka::RobotState &robot_state, double time) {
      return motion_func_(robot_state, time);
    }

    bool condition(const franka::RobotState &robot_state, double time) {
      return condition_(robot_state, time);
    }

  private:
    MotionFunc motion_func_;
    Condition condition_;
  };
} // namespace franky
