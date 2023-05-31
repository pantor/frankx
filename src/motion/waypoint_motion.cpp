#include "franky/motion/waypoint_motion.hpp"

#include <ruckig/ruckig.hpp>
#include <utility>

#include "franky/robot_pose.hpp"
#include "franky/waypoint.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"

namespace franky {

WaypointMotion::WaypointMotion(const std::vector<Waypoint> &waypoints)
    : WaypointMotion(waypoints, Params()) {}

WaypointMotion::WaypointMotion(std::vector<Waypoint> waypoints, WaypointMotion::Params params)
    : waypoints_(std::move(waypoints)), params_(std::move(params)), prev_result_() {}

void WaypointMotion::initImpl(const franka::RobotState &robot_state) {
  franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
  RobotPose robot_pose(initial_cartesian_pose);
  ref_frame_ = Affine::Identity();
  current_cooldown_iteration_ = 0;

  auto initial_velocity = Vector<6>::Map(robot_state.O_dP_EE_c.data());
  Vector7d initial_velocity_with_elbow = (Vector7d() << initial_velocity, robot_state.delbow_c[0]).finished();

  auto initial_acceleration = Vector<6>::Map(robot_state.O_ddP_EE_c.data());
  Vector7d initial_acceleration_with_elbow = (Vector7d() << initial_acceleration, robot_state.ddelbow_c[0]).finished();

  input_para_.current_position = toStd<7>(robot_pose.vector_repr());
  input_para_.current_velocity = toStd<7>(initial_velocity_with_elbow);
  input_para_.current_acceleration = toStd<7>(initial_acceleration_with_elbow);
  waypoint_has_elbow_ = true;

  target_robot_pose_ = RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_);
  waypoint_iterator_ = waypoints_.begin();
  if (waypoint_iterator_ != waypoints_.end()) {
    setNewWaypoint(robot_state, *waypoint_iterator_);
    prev_result_ = ruckig::Result::Working;
  } else {
    prev_result_ = ruckig::Result::Finished;
  }
}

franka::CartesianPose
WaypointMotion::nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
  const uint64_t steps = std::max<uint64_t>(time_step.toMSec(), 1);
  for (size_t i = 0; i < steps; i++) {
    if (prev_result_ == ruckig::Result::Finished) {
      if (waypoint_iterator_ != waypoints_.end())
        ++waypoint_iterator_;
      if (waypoint_iterator_ == waypoints_.end()) {
        auto output_pose = (
            ref_frame_ * RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_)).as_franka_pose();
        // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
        if (!params_.return_when_finished) {
          return output_pose;
        } else if (current_cooldown_iteration_ < cooldown_iterations_) {
          current_cooldown_iteration_ += 1;
          return output_pose;
        }
        return franka::MotionFinished(output_pose);
      } else {
        setNewWaypoint(robot_state, *waypoint_iterator_);
      }
    }
    if (waypoint_iterator_ != waypoints_.end()) {
      prev_result_ = trajectory_generator_.update(input_para_, output_para_);
      if (prev_result_ == ruckig::Result::Error) {
        throw std::runtime_error("Invalid inputs to motion planner.");
      }
      output_para_.pass_to_input(input_para_);
    }
  }

  return (ref_frame_ * RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_)).as_franka_pose();
}

void WaypointMotion::setNewWaypoint(const franka::RobotState &robot_state, const Waypoint &new_waypoint) {
  // We first convert the current state into the frame of the current pose
  RobotPose current_pose_old_ref_frame = RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_);
  Affine new_ref_to_old_ref = current_pose_old_ref_frame.end_effector_pose();
  ref_frame_ = ref_frame_ * new_ref_to_old_ref;
  auto rot = new_ref_to_old_ref.inverse().rotation();

  Vector<7> current_velocity = toEigen<7>(input_para_.current_velocity);
  auto linear_vel_ref_frame = rot * current_velocity.head<3>();
  auto angular_vel_ref_frame = rot * current_velocity.segment<3>(3);

  Vector<7> current_acc = toEigen<7>(input_para_.current_acceleration);
  auto linear_acc_ref_frame = rot * current_acc.head<3>();
  auto angular_acc_ref_frame = rot * current_acc.segment<3>(3);

  double elbow_velocity, elbow_acc;
  if (waypoint_has_elbow_) {
    elbow_velocity = current_velocity[6];
    elbow_acc = current_acc[6];
  } else {
    elbow_velocity = robot_state.delbow_c[6];
    elbow_acc = robot_state.ddelbow_c[6];
  }

  RobotPose zero_pose(Affine::Identity(), current_pose_old_ref_frame.elbow_position());
  Vector7d current_velocity_ref_frame =
      (Vector7d() << linear_vel_ref_frame, angular_vel_ref_frame, elbow_velocity).finished();
  Vector7d current_acc_ref_frame =
      (Vector7d() << linear_acc_ref_frame, angular_acc_ref_frame, elbow_acc).finished();
  input_para_.current_position = toStd<7>(zero_pose.vector_repr());
  input_para_.current_velocity = toStd<7>(current_velocity_ref_frame);
  input_para_.current_acceleration = toStd<7>(current_acc_ref_frame);

  waypoint_has_elbow_ = new_waypoint.robot_pose.elbow_position().has_value();

  auto prev_target_robot_pose = target_robot_pose_;
  if (!target_robot_pose_.elbow_position().has_value()) {
    prev_target_robot_pose = prev_target_robot_pose.with_elbow_position(robot_state.elbow[0]);
  }
  auto new_target_robot_pose = new_waypoint.getTargetRobotPose(prev_target_robot_pose) * params_.frame.inverse();
  auto new_target_robot_pose_ref_frame = ref_frame_.inverse() * new_target_robot_pose;
  input_para_.enabled = {true, true, true, true, true, true, waypoint_has_elbow_};
  input_para_.target_position = toStd<7>(new_target_robot_pose_ref_frame.vector_repr());
  input_para_.target_velocity = toStd<7>(Vector7d::Zero());
  setInputLimits(input_para_, new_waypoint);

  target_robot_pose_ = new_target_robot_pose;
}

std::tuple<Vector7d, Vector7d, Vector7d>
WaypointMotion::getInputLimits(const Waypoint &waypoint) const {
  constexpr double translation_factor{0.4};
  constexpr double derivative_factor{0.4};
  const Robot *robot = this->robot();

  Vector7d max_vel = vec_cart_rot_elbow(
      translation_factor * Robot::max_translation_velocity,
      Robot::max_rotation_velocity,
      Robot::max_elbow_velocity);
  Vector7d max_acc = derivative_factor * vec_cart_rot_elbow(
      translation_factor * Robot::max_elbow_acceleration,
      Robot::max_elbow_acceleration,
      Robot::max_elbow_acceleration);
  Vector7d max_jerk = std::pow(derivative_factor, 2) * vec_cart_rot_elbow(
      translation_factor * Robot::max_translation_jerk,
      Robot::max_rotation_jerk,
      Robot::max_elbow_jerk);

  double vel_factor, acc_factor, jerk_factor;

  if (waypoint.max_dynamics || params_.max_dynamics) {
    vel_factor = 1.0;
    acc_factor = 1.0;
    jerk_factor = 1.0;
  } else {
    vel_factor = waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel();
    acc_factor = params_.acceleration_rel * robot->acceleration_rel();
    jerk_factor = params_.jerk_rel * robot->jerk_rel();
  }

  auto vel_lim = vel_factor * max_vel;
  auto acc_lim = acc_factor * max_acc;
  auto jerk_lim = jerk_factor * max_jerk;
  return {vel_lim, acc_lim, jerk_lim};
}

void WaypointMotion::setInputLimits(ruckig::InputParameter<7> &input_parameters, const Waypoint &waypoint) const {
  auto [vel_lim, acc_lim, jerk_lim] = getInputLimits(waypoint);
  input_parameters.max_velocity = toStd<7>(vel_lim);
  input_parameters.max_acceleration = toStd<7>(acc_lim);
  input_parameters.max_jerk = toStd<7>(jerk_lim);

  if (!(waypoint.max_dynamics || params_.max_dynamics) && waypoint.minimum_time.has_value()) {
    input_parameters.minimum_duration = waypoint.minimum_time.value();
  }

  if (waypoint.max_dynamics) {
    input_parameters.synchronization = ruckig::Synchronization::TimeIfNecessary;
  } else {
    input_parameters.synchronization = ruckig::Synchronization::Time;
  }
}

}  // namespace franky
