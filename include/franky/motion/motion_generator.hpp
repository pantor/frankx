#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>
#include "ruckig/ruckig.hpp"

#include "franky/types.hpp"
#include "franky/robot.hpp"
#include "franky/motion/motion.hpp"

namespace franky {

class Robot;

template<typename ControlSignalType>
class Motion;

template<typename ControlSignalType>
class MotionGenerator {
 public:
  static constexpr size_t REACTION_RECURSION_LIMIT = 8;

  explicit MotionGenerator(Robot *robot, std::shared_ptr<Motion<ControlSignalType>> initial_motion);

  ControlSignalType operator()(const franka::RobotState &robot_state, franka::Duration period);

  inline void
  registerUpdateCallback(const std::function<void(const franka::RobotState &, franka::Duration, double)> &callback) {
    update_callbacks_.push_back(callback);
  }

 private:
  std::shared_ptr<Motion<ControlSignalType>> initial_motion_;
  std::shared_ptr<Motion<ControlSignalType>> current_motion_;
  std::vector<std::function<void(const franka::RobotState &, franka::Duration, double)>> update_callbacks_;

  double time_{0.0};
  Robot *robot_;
};

}  // namespace franky
