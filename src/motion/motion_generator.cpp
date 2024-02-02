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
  abs_time_ += period.toSec();
  {
    if (abs_time_ == 0.0)
      previous_command_ = std::nullopt;
    std::unique_lock<std::mutex> lock(new_motion_mutex_);
    if (new_motion_ != nullptr || abs_time_ == 0.0) {
      if (new_motion_ != nullptr) {
        current_motion_ = new_motion_;
        new_motion_ = nullptr;
      } else {
        current_motion_ = initial_motion_;
      }
      rel_time_offset_ = abs_time_;
      current_motion_->init(robot_, robot_state, previous_command_);
    }
  }

  auto rel_time = abs_time_ - rel_time_offset_;
  for (auto &callback : update_callbacks_)
    callback(robot_state, period, rel_time);

  size_t recursion_depth = 0;
  bool reaction_fired = true;
  while (reaction_fired) {
    reaction_fired = false;
    auto new_motion = current_motion_->checkAndCallReactions(robot_state, abs_time_ - rel_time_offset_, abs_time_);
    if (new_motion != nullptr) {
      current_motion_ = new_motion;
      current_motion_->init(robot_, robot_state, previous_command_);
      rel_time_offset_ = abs_time_;
      reaction_fired = true;
      recursion_depth++;
      if (recursion_depth > REACTION_RECURSION_LIMIT) {
        throw ReactionRecursionException(
            "Reaction recursion reached depth limit " + std::to_string(REACTION_RECURSION_LIMIT) + ".");
      }
    }
  }
  previous_command_ = current_motion_->nextCommand(robot_state, period, rel_time, abs_time_, previous_command_);
  return previous_command_.value();
}

template<typename ControlSignalType>
void MotionGenerator<ControlSignalType>::updateMotion(const std::shared_ptr<Motion<ControlSignalType>> new_motion) {
  std::unique_lock<std::mutex> lock(new_motion_mutex_);
  new_motion_ = new_motion;
}

template<typename ControlSignalType>
bool MotionGenerator<ControlSignalType>::has_new_motion() {
  return new_motion_ != nullptr;
}

template<typename ControlSignalType>
void MotionGenerator<ControlSignalType>::resetTimeUnsafe() {
  abs_time_ = 0.0;
  rel_time_offset_ = 0.0;
}

}  // namespace franky
