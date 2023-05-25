#include "franky/motion/motion.hpp"

#include <mutex>

#include "franky/motion/reaction.hpp"
#include "franky/robot.hpp"

namespace franky {
  template class Motion<franka::Torques>;
  template class Motion<franka::JointVelocities>;
  template class Motion<franka::CartesianVelocities>;
  template class Motion<franka::JointPositions>;
  template class Motion<franka::CartesianPose>;

  template<typename ControlSignalType>
  Motion<ControlSignalType>::Motion() : robot_(nullptr) {}

  template<typename ControlSignalType>
  void Motion<ControlSignalType>::addReaction(const std::shared_ptr<Reaction<ControlSignalType>> reaction) {
    const std::lock_guard<std::mutex> lock(mutex_);
    reactions_.push_back(reaction);
  }

  template<typename ControlSignalType>
  std::vector<std::shared_ptr<Reaction<ControlSignalType>>> Motion<ControlSignalType>::reactions() {
    const std::lock_guard<std::mutex> lock(mutex_);
    return reactions_;
  }

  template<typename ControlSignalType>
  void Motion<ControlSignalType>::init(Robot *robot, const franka::RobotState &robot_state, double time) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = robot;
    initImpl(robot_state, time);
  }

  template<typename ControlSignalType>
  ControlSignalType
  Motion<ControlSignalType>::nextCommand(const franka::RobotState &robot_state, franka::Duration time_step,
                                         double time) {
    std::lock_guard<std::mutex> lock(mutex_);
    return nextCommandImpl(robot_state, time_step, time);
  }
}  // namespace franky
