#pragma once

#include <mutex>

#include "reaction.hpp"
#include "franky/robot.hpp"

namespace franky {
  class Robot;

  template<typename ControlSignalType>
  class Reaction;

  template<typename ControlSignalType>
  class Motion {
  public:
    explicit Motion() : robot_(nullptr) {}

    void addReaction(const std::shared_ptr<Reaction<ControlSignalType>> reaction) {
      const std::lock_guard<std::mutex> lock(mutex_);
      reactions_.push_back(reaction);
    }

    std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions() {
      const std::lock_guard<std::mutex> lock(mutex_);
      return reactions_;
    }

    void init(Robot *robot, const franka::RobotState &robot_state, double time) {
      std::lock_guard<std::mutex> lock(mutex_);
      robot_ = robot;
      initImpl(robot_state, time);
    };

    ControlSignalType
    nextCommand(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
      std::lock_guard<std::mutex> lock(mutex_);
      return nextCommandImpl(robot_state, time_step, time);
    };

  protected:
    virtual void initImpl(const franka::RobotState &robot_state, double time) {}

    virtual ControlSignalType
    nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

    Robot *robot() const {
      return robot_;
    }

  private:
    std::mutex mutex_;
    std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions_;
    Robot *robot_;
  };
} // namespace franky
