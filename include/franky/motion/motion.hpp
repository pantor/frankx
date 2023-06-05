#pragma once

#include <mutex>
#include <list>

#include "reaction.hpp"
#include "franky/robot.hpp"

namespace franky {

class Robot;

template<typename ControlSignalType>
class Reaction;

template<typename ControlSignalType>
class Motion {
 public:
  using CallbackType = std::function<
      void(const franka::RobotState &, franka::Duration, double, double, const ControlSignalType &)>;

  void addReaction(std::shared_ptr<Reaction<ControlSignalType>> reaction);

  void addReactionFront(std::shared_ptr<Reaction<ControlSignalType>> reaction);

  std::vector<std::shared_ptr<Reaction<ControlSignalType>>> reactions();

  void registerCallback(CallbackType callback);

  void init(Robot *robot, const franka::RobotState &robot_state);

  ControlSignalType
  nextCommand(const franka::RobotState &robot_state, franka::Duration time_step, double rel_time, double abs_time);

  std::shared_ptr<Motion<ControlSignalType>>
  checkAndCallReactions(const franka::RobotState &robot_state, double rel_time, double abs_time);

 protected:
  explicit Motion();

  virtual void initImpl(const franka::RobotState &robot_state) {}

  virtual ControlSignalType
  nextCommandImpl(
      const franka::RobotState &robot_state, franka::Duration time_step, double rel_time, double abs_time) = 0;

  [[nodiscard]] inline Robot *robot() const {
    return robot_;
  }

 private:
  std::mutex reaction_mutex_;
  std::list<std::shared_ptr<Reaction<ControlSignalType>>> reactions_;
  std::mutex callback_mutex_;
  std::vector<CallbackType> callbacks_;
  Robot *robot_;
};

}  // namespace franky
