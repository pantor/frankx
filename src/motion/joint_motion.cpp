#include "franky/motion/joint_motion.hpp"

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>
#include <utility>
#include <franka/robot_state.h>
#include <franka/control_types.h>

#include "franky/util.hpp"
#include "franky/types.hpp"

namespace franky {

JointMotion::JointMotion(const Vector7d &target) : JointMotion(target, Params()) {}

JointMotion::JointMotion(Vector7d target, const JointMotion::Params &params)
    : target_(std::move(target)), params_(params) {}

void JointMotion::initImpl(const franka::RobotState &robot_state) {
  current_cooldown_iteration_ = 0;

  input_para_.current_position = robot_state.q_d;
  input_para_.current_velocity = toStd<7>(Vector7d::Zero());
  input_para_.current_acceleration = toStd<7>(Vector7d::Zero());

  input_para_.target_position = toStd<7>(target_);
  input_para_.target_velocity = toStd<7>(Vector7d::Zero());
  input_para_.target_acceleration = toStd<7>(Vector7d::Zero());

  for (size_t dof = 0; dof < 7; dof++) {
    auto velocity_rel = params_.max_dynamics ? 1.0 : robot()->velocity_rel() * params_.velocity_rel;
    auto acceleration_rel = params_.max_dynamics ? 1.0 : robot()->acceleration_rel() * params_.acceleration_rel;
    auto jerk_rel = params_.max_dynamics ? 1.0 : robot()->jerk_rel() * params_.jerk_rel;
    input_para_.max_velocity[dof] = Robot::max_joint_velocity[dof] * velocity_rel;
    input_para_.max_acceleration[dof] = 0.3 * Robot::max_joint_acceleration[dof] * acceleration_rel;
    input_para_.max_jerk[dof] = 0.3 * Robot::max_joint_jerk[dof] * jerk_rel;
  }
}

franka::JointPositions
JointMotion::nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
  std::array<double, 7> joint_positions{};
  const uint64_t steps = std::max<uint64_t>(time_step.toMSec(), 1);
  for (size_t i = 0; i < steps; i++) {
    auto result = trajectory_generator_.update(input_para_, output_para_);
    joint_positions = output_para_.new_position;
    if (result == ruckig::Result::Finished) {
      joint_positions = input_para_.target_position;
      // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
      auto output_pose = franka::JointPositions(joint_positions);
      if (params_.return_when_finished) {
        if (current_cooldown_iteration_ < cooldown_iterations_) {
          current_cooldown_iteration_ += 1;
          return output_pose;
        }
        return franka::MotionFinished(output_pose);
      }
      return output_pose;
    } else if (result == ruckig::Result::Error) {
      throw std::runtime_error("Invalid inputs to motion planner.");
    }
    output_para_.pass_to_input(input_para_);
  }
  return {joint_positions};
}

}  // namespace franky
