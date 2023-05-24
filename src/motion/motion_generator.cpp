#include "franky/motion/motion_generator.hpp"

#include <franka/duration.h>
#include <franka/robot_state.h>

#include "franky/robot.hpp"
#include "franky/motion/motion.hpp"


namespace franky {
  template class MotionGenerator<franka::Torques>;
  template class MotionGenerator<franka::JointVelocities>;
  template class MotionGenerator<franka::CartesianVelocities>;
  template class MotionGenerator<franka::JointPositions>;
  template class MotionGenerator<franka::CartesianPose>;

  template<typename ControlSignalType>
  MotionGenerator<ControlSignalType>::MotionGenerator(
      Robot *robot, std::shared_ptr<Motion<ControlSignalType>> initial_motion)
      : robot_(robot), initial_motion_(initial_motion) {}

  template<typename ControlSignalType>
  ControlSignalType
  MotionGenerator<ControlSignalType>::operator()(const franka::RobotState &robot_state, franka::Duration period) {
    if (time_ == 0.0) {
      current_motion_ = initial_motion_;
      current_motion_->init(robot_, robot_state, time_);
    }
    time_ += period.toSec();

    for (auto &callback: update_callbacks_)
      callback(robot_state, period, time_);

    size_t recursion_depth = 0;
    bool reaction_fired = true;
    while (reaction_fired) {
      reaction_fired = false;
      for (auto &reaction: current_motion_->reactions()) {
        if (reaction->condition(robot_state, time_)) {
          current_motion_ = (*reaction)(robot_state, time_);
          current_motion_->init(robot_, robot_state, time_);
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
    return current_motion_->nextCommand(robot_state, period, time_);
  }
} // namespace franky
