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
  using MotionFunc = std::function<std::shared_ptr<Motion<ControlSignalType>>(const franka::RobotState &, double, double)>;

 public:
  explicit Reaction(const Condition &condition, std::shared_ptr<Motion<ControlSignalType>> new_motion = nullptr);

  explicit Reaction(Condition condition, const MotionFunc &motion_func);

  std::shared_ptr<Motion<ControlSignalType>> operator()(const franka::RobotState &robot_state, double rel_time, double abs_time);

  [[nodiscard]] inline bool condition(const franka::RobotState &robot_state, double rel_time, double abs_time) const {
    return condition_(robot_state, rel_time, abs_time);
  }

  void registerCallback(std::function<void(const franka::RobotState &, double, double)> callback);

 private:
  MotionFunc motion_func_;
  Condition condition_;
  std::mutex callback_mutex_;
  std::vector<std::function<void(const franka::RobotState &, double, double)>> callbacks_{};
};

}  // namespace franky
