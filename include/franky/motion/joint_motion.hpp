#pragma once

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/types.hpp"
#include "franky/motion/motion.hpp"

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

  explicit JointMotion(const Vector7d &target);

  explicit JointMotion(Vector7d target, const Params &params);

 protected:
  void initImpl(const franka::RobotState &robot_state, double time) override;

  franka::JointPositions
  nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

 private:
  Vector7d target_;
  Params params_;

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::InputParameter<7> input_para;
  ruckig::OutputParameter<7> output_para;

  ruckig::Result result;

  std::array<double, 7> joint_positions{};

  constexpr static size_t cooldown_iterations_{5};
  size_t current_cooldown_iteration_{0};
};

}  // namespace franky
