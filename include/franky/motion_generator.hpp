#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>
#include <ruckig/ruckig.hpp>

#include "franky/types.hpp"
#include "franky/robot.hpp"
#include "franky/motion/motion.hpp"


namespace franky {
  template<typename ControlSignalType>
  class MotionGenerator {
    friend class Motion<ControlSignalType>;

    explicit MotionGenerator(Robot *robot, std::shared_ptr<Motion<ControlSignalType>> initial_motion,
                              bool stop_at_python_signal = true)
        : robot_(robot), initial_motion_(initial_motion), stop_at_python_signal_(stop_at_python_signal) {}

    franka::CartesianPose operator()(const franka::RobotState &robot_state, franka::Duration period) {
      std::lock_guard<std::mutex> lock(current_motion_.mutex());
      if (time_ == 0.0) {
        current_motion_ = initial_motion_;
        current_motion_->initUnsafe(robot_state);
      }
      time_ += period.toSec();

      asynchronous_state_ = robot_state;

      for (auto &reaction: current_motion_->reactions_) {
        if (reaction.condition(robot_state, time_)) {
          current_motion_ = reaction(robot_state, time_);
          current_motion_->initUnsafe();
        }
      }
      return current_motion_->nextCommandUnsafe(robot_state, period, time_);
    }

  private:
    std::shared_ptr<Motion<ControlSignalType>> initial_motion_;
    std::shared_ptr<Motion<ControlSignalType>> current_motion_;
    bool stop_at_python_signal_;

    double time_{0.0};
    Robot *robot_;

    franka::RobotState asynchronous_state_;
  };
} // namespace franky
