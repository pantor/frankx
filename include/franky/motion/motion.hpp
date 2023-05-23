#pragma once

#include <mutex>

#include "franky/robot/reaction.hpp"
#include "franky/robot.hpp"

namespace franky {
  class Robot;

  template<typename ControlSignalType>
  class Reaction;

  template<typename ControlSignalType>
  class Motion {
  public:
    explicit Motion() : robot_(nullptr) {}


    void init(Robot* robot, const franka::RobotState &robot_state) {
      robot_ = robot;
      on_init(robot_state);
    };

    virtual ControlSignalType
    next_command(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

    void add_reaction(const std::shared_ptr<Reaction<ControlSignalType>> reaction) {
      reactions_.push_back(reaction);
    }

    const std::vector<std::shared_ptr<Reaction<ControlSignalType>>> &reactions() {
      return reactions_;
    }

    const std::mutex &access_mutex() {
      return access_mutex_;
    }

  protected:
    void on_init(const franka::RobotState &robot_state) {};

    Robot *robot() const {
      return robot_;
    }

  private:
    std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions_;
    std::mutex access_mutex_;
    Robot *robot_;
  };
} // namespace franky
