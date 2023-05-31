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
  void addReaction(std::shared_ptr<Reaction<ControlSignalType>> reaction);

  std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions();

  void init(Robot *robot, const franka::RobotState &robot_state);

  ControlSignalType
  nextCommand(const franka::RobotState &robot_state, franka::Duration time_step, double time);

  std::shared_ptr<Motion<ControlSignalType>>
  checkAndCallReactions(const franka::RobotState &robot_state, double rel_time, double abs_time_);

 protected:
  explicit Motion();

  virtual void initImpl(const franka::RobotState &robot_state) {}

  virtual ControlSignalType
  nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

  [[nodiscard]] inline Robot *robot() const {
    return robot_;
  }

 private:
  std::mutex reaction_mutex_;
  std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions_;
  Robot *robot_;
};

}  // namespace franky
