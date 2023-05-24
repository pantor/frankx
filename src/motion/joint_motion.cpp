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

  JointMotion::JointMotion(Vector7d target, const JointMotion::Params &params) : target_(std::move(target)), params_(params) {}

  void JointMotion::initImpl(const franka::RobotState &robot_state, double time) {
    current_cooldown_iteration_ = 0;

    input_para.current_position = robot_state.q_d;
    input_para.current_velocity = toStd<7>(Vector7d::Zero());
    input_para.current_acceleration = toStd<7>(Vector7d::Zero());

    input_para.target_position = toStd<7>(target_);
    input_para.target_velocity = toStd<7>(Vector7d::Zero());
    input_para.target_acceleration = toStd<7>(Vector7d::Zero());

    for (size_t dof = 0; dof < 7; dof++) {
      input_para.max_velocity[dof] = Robot::max_joint_velocity[dof] * robot()->velocity_rel() * params_.velocity_rel;
      input_para.max_acceleration[dof] =
          0.3 * Robot::max_joint_acceleration[dof] * robot()->acceleration_rel() * params_.acceleration_rel;
      input_para.max_jerk[dof] = 0.3 * Robot::max_joint_jerk[dof] * robot()->jerk_rel() * params_.jerk_rel;
    }
  }

  franka::JointPositions
  JointMotion::nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
    const uint64_t steps = std::max<uint64_t>(time_step.toMSec(), 1);
    for (size_t i = 0; i < steps; i++) {
      result = trajectory_generator_.update(input_para, output_para);
      joint_positions = output_para.new_position;
      if (result == ruckig::Result::Finished) {
        joint_positions = input_para.target_position;
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
      output_para.pass_to_input(input_para);
    }
    return {joint_positions};
  }

}  // namespace franky
