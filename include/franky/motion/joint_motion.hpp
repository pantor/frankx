#pragma once

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"


namespace franky {

/**
* A motion in the joint space
*/
  class JointMotion : public Motion<franka::JointPositions> {

  public:
    struct Params {
      double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};
      bool return_when_finished{true};
    };

    explicit JointMotion(const Vector7d &target) : JointMotion(target, Params()) {}

    explicit JointMotion(const Vector7d &target, const Params &params) : target_(target), params_(params) {}

  private:
    Vector7d target_;
    Params params_;

    ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
    ruckig::InputParameter<7> input_para;
    ruckig::OutputParameter<7> output_para;

    ruckig::Result result;

    std::array<double, 7> joint_positions;

    constexpr static size_t cooldown_iterations_{5};
    size_t current_cooldown_iteration_{0};

    void initImpl(const franka::RobotState &robot_state, double time) {
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
    nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
      const int steps = std::max<int>(time_step.toMSec(), 1);
      for (int i = 0; i < steps; i++) {
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
      return franka::JointPositions(joint_positions);
    }
  };

} // namespace franky
