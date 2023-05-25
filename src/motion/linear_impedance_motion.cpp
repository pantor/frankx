#include "franky/motion/linear_impedance_motion.hpp"

#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/motion/impedance_motion.hpp"

namespace franky {

LinearImpedanceMotion::LinearImpedanceMotion(const Affine &target, double duration)
    : LinearImpedanceMotion(target, duration, Params()) {}

LinearImpedanceMotion::LinearImpedanceMotion(
    const Affine &target, double duration, const LinearImpedanceMotion::Params &params)
    : duration_(duration), params_(params), ImpedanceMotion(target, params) {}

void LinearImpedanceMotion::initImpl(const franka::RobotState &robot_state, double time) {
  ImpedanceMotion::initImpl(robot_state, time);
  initial_pose_ = Affine(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  motion_init_time_ = time;
}

std::tuple<Affine, bool>
LinearImpedanceMotion::update(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
  double transition_parameter = (time - motion_init_time_) / duration_;
  Affine intermediate_goal;
  bool done;
  if (transition_parameter <= 1.0) {  // [ms] to [s]
    Eigen::Quaterniond q_start(initial_pose_.rotation());
    Eigen::Quaterniond q_end(target().rotation());
    auto init_trans = initial_pose_.translation();
    auto trans = init_trans + transition_parameter * (target().translation() - init_trans);
    auto rot = q_start.slerp(transition_parameter, q_end);
    intermediate_goal.fromPositionOrientationScale(trans, rot, Eigen::Vector3d::Ones());
    done = false;
  } else if (params_.return_when_finished && transition_parameter > params_.finish_wait_factor) {
    done = true;
    intermediate_goal = target();
  }
  return {intermediate_goal, done};
}

}  // namespace franky
