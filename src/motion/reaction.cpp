#include "franky/motion/reaction.hpp"

#include <memory>
#include <utility>
#include <franka/robot_state.h>

#include "franky/motion/motion.hpp"
#include "franky/motion/condition.hpp"

namespace franky {

template class Reaction<franka::Torques>;
template class Reaction<franka::JointVelocities>;
template class Reaction<franka::CartesianVelocities>;
template class Reaction<franka::JointPositions>;
template class Reaction<franka::CartesianPose>;

template<typename ControlSignalType>
Reaction<ControlSignalType>::Reaction(
    const Condition &condition, const std::shared_ptr<Motion<ControlSignalType>> new_motion)
    : Reaction(condition, [new_motion](const franka::RobotState &, double) { return new_motion; }) {}

template<typename ControlSignalType>
Reaction<ControlSignalType>::Reaction(Condition condition, const Reaction::MotionFunc &motion_func)
    : condition_(std::move(condition)), motion_func_(motion_func) {}

}  // namespace franky
