#pragma once

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>
#include <franka/robot_state.h>

#include "franky/motion/motion.hpp"
#include "franky/motion/condition.hpp"


namespace franky {
  template<typename ControlSignalType>
  class Motion;

  template<typename ControlSignalType>
  class Reaction {
    using MotionFunc = std::function<std::shared_ptr<Motion<ControlSignalType>>(const franka::RobotState &, double)>;

  public:
    explicit Reaction(const Condition &condition, const std::shared_ptr<Motion<ControlSignalType>> new_motion);

    explicit Reaction(const Condition &condition, const MotionFunc &motion_func);

    inline std::shared_ptr<Motion<ControlSignalType>> operator()(const franka::RobotState &robot_state, double time) {
      return motion_func_(robot_state, time);
    }

    inline bool condition(const franka::RobotState &robot_state, double time) {
      return condition_(robot_state, time);
    }

  private:
    MotionFunc motion_func_;
    Condition condition_;
  };
} // namespace franky
