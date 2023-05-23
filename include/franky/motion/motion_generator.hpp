#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>
#include "ruckig/ruckig.hpp"

#include "franky/types.hpp"
#include "franky/robot.hpp"
#include "franky/motion/motion.hpp"


namespace franky {
  template<typename ControlSignalType>
  class MotionGenerator {
    friend class Motion<ControlSignalType>;

  public:
    static constexpr size_t REACTION_RECURSION_LIMIT = 8;

    explicit MotionGenerator(Robot *robot, std::shared_ptr<Motion<ControlSignalType>> initial_motion)
        : robot_(robot), initial_motion_(initial_motion) {}

    franka::CartesianPose operator()(const franka::RobotState &robot_state, franka::Duration period) {
      std::lock_guard<std::mutex> lock(current_motion_.mutex());
      if (time_ == 0.0) {
        current_motion_ = initial_motion_;
        current_motion_->initUnsafe(robot_, robot_state, time_);
      }
      time_ += period.toSec();

      for(auto &callback: update_callbacks_)
        callback(robot_state, period, time_);

      size_t recursion_depth = 0;
      bool reaction_fired = true;
      while (reaction_fired) {
        reaction_fired = false;
        for (auto &reaction: current_motion_->reactions_) {
          if (reaction.condition(robot_state, time_)) {
            current_motion_ = reaction(robot_state, time_);
            current_motion_->initUnsafe(robot_state, time_);
            reaction_fired = true;
            recursion_depth++;
            break;
          }
        }
        if (recursion_depth > REACTION_RECURSION_LIMIT) {
          throw std::runtime_error(
              "Reaction recursion reached depth limit " + std::to_string(REACTION_RECURSION_LIMIT) + ".");
        }
      }
      return current_motion_->nextCommandUnsafe(robot_state, period, time_);
    }

    void
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
} // namespace franky
