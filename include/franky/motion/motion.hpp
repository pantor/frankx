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
    explicit Motion();

    void addReaction(std::shared_ptr<Reaction<ControlSignalType>> reaction);

    std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions();

    void init(Robot *robot, const franka::RobotState &robot_state, double time);;

    ControlSignalType
    nextCommand(const franka::RobotState &robot_state, franka::Duration time_step, double time);;

  protected:
    virtual void initImpl(const franka::RobotState &robot_state, double time) {}

    virtual ControlSignalType
    nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

    [[nodiscard]] inline Robot *robot() const {
      return robot_;
    }

  private:
    std::mutex mutex_;
    std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions_;
    Robot *robot_;
  };
}  // namespace franky
