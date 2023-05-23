#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>


#include "franky/robot_pose.hpp"
#include "franky/waypoint.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"
#include "franky/motion/motion.hpp"


namespace franky {

/**
* A motion following multiple waypoints (with intermediate zero velocity) in a time-optimal way.
* Works with arbitrary initial conditions.
*/
  class WaypointMotion : public Motion<franka::CartesianPose> {
  public:
    struct Params {
      double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};
      bool max_dynamics{false};
      bool return_when_finished{true};
    };

    explicit WaypointMotion(std::vector<Waypoint> waypoints)
        : WaypointMotion(waypoints, Params()) {}

    explicit WaypointMotion(std::vector<Waypoint> waypoints, const Params &params)
        : waypoints_(waypoints), params_(params) {}

    void initImpl(const franka::RobotState &robot_state) {
      franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
      RobotPose robot_pose(initial_cartesian_pose);
      ref_frame_ = Affine();

      RobotPose zero_pose(Affine::Identity(), robot_pose.elbow_position().value());
      auto initial_velocity = Vector<6>::Map(robot_state.O_dP_EE_c.data());
      Vector7d initial_velocity_with_elbow = (Vector7d() << initial_velocity, robot_state.delbow_c[0]).finished();
      input_para_.current_position = toStd<7>(robot_pose.vector_repr());
      input_para_.current_velocity = toStd<7>(initial_velocity_with_elbow);
      input_para_.current_acceleration = toStd<7>(Vector7d::Zero());
      waypoint_has_elbow_ = true;

      target_robot_pose_ = ref_frame_ * RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_);
      waypoint_iterator_ = waypoints_.begin();
      if (waypoint_iterator_ != waypoints_.end())
        setNewWaypoint(robot_state, *waypoint_iterator_);
    }

    franka::CartesianPose nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
      const int steps = std::max<int>(time_step.toMSec(), 1);
      for (int i = 0; i < steps; i++) {
        result_ = trajectory_generator_.update(input_para_, output_para_);

        if (result_ == ruckig::Result::Finished) {
          bool has_new_waypoint = false;

          if (waypoint_iterator_ != waypoints_.end()) {
            ++waypoint_iterator_;
            has_new_waypoint = (waypoint_iterator_ != waypoints_.end());
          }

          if (!has_new_waypoint && params_.return_when_finished) {
            auto output_pose = (
                ref_frame_ *
                RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_)).as_franka_pose();
            // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
            if (current_cooldown_iteration < cooldown_iterations) {
              current_cooldown_iteration += 1;
              return output_pose;
            }
            return franka::MotionFinished(output_pose);
          }

          if (has_new_waypoint) {
            setNewWaypoint(robot_state, *waypoint_iterator_);
          }

        } else if (result_ == ruckig::Result::Error) {
          throw std::runtime_error("Invalid inputs to motion planner.");
        }
        output_para_.pass_to_input(input_para_);
      }
      return (ref_frame_ * RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_)).as_franka_pose();
    }


  private:
    std::vector<Waypoint> waypoints_;
    Params params_;

    RobotPose target_robot_pose_;

    Affine ref_frame_;

    ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
    ruckig::InputParameter<7> input_para_;
    ruckig::OutputParameter<7> output_para_;
    ruckig::Result result_;

    std::vector<Waypoint>::iterator waypoint_iterator_;
    bool waypoint_has_elbow_{false};

    Affine frame_;

    constexpr static size_t cooldown_iterations{5};
    size_t current_cooldown_iteration{0};

    void setNewWaypoint(const franka::RobotState &robot_state, const Waypoint &new_waypoint) {
      // We first convert the current state into the frame of the current pose
      RobotPose current_pose = RobotPose(toEigen<7>(input_para_.current_position), !waypoint_has_elbow_);
      Affine current_pose_ref_frame = ref_frame_.inverse() * current_pose.end_effector_pose();
      ref_frame_ = current_pose.end_effector_pose();
      auto rot = current_pose_ref_frame.rotation().transpose();

      Vector<7> current_velocity = toEigen<7>(input_para_.current_velocity);
      auto linear_vel_ref_frame = rot * current_velocity.head<3>();
      auto angular_vel_ref_frame = rot * current_velocity.segment<3>(3);

      Vector<7> current_acc = toEigen<7>(input_para_.current_position);
      auto linear_acc_ref_frame = rot * current_acc.head<3>();
      auto angular_acc_ref_frame = rot * current_acc.segment<3>(3);

      RobotPose zero_pose(Affine::Identity(), current_pose.elbow_position());
      Vector7d current_velocity_ref_frame =
          (Vector7d() << linear_vel_ref_frame, angular_vel_ref_frame, current_velocity[6]).finished();
      Vector7d current_acc_ref_frame =
          (Vector7d() << linear_acc_ref_frame, angular_acc_ref_frame, current_acc[6]).finished();
      input_para_.current_position = toStd<7>(zero_pose.vector_repr());
      input_para_.current_velocity = toStd<7>(current_velocity_ref_frame);
      input_para_.current_acceleration = toStd<7>(current_acc_ref_frame);

      waypoint_has_elbow_ = new_waypoint.robot_pose.elbow_position().has_value();

      auto prev_target_robot_pose = target_robot_pose_;
      if (!target_robot_pose_.elbow_position().has_value()) {
        prev_target_robot_pose = prev_target_robot_pose.with_elbow_position(robot_state.elbow[0]);
      }
      auto new_target_robot_pose = new_waypoint.getTargetRobotPose(prev_target_robot_pose) * frame_.inverse();
      auto new_target_robot_pose_ref_frame = ref_frame_.inverse() * new_target_robot_pose;
      input_para_.enabled = {true, true, true, true, true, true, waypoint_has_elbow_};
      input_para_.target_position = toStd<7>(new_target_robot_pose_ref_frame.vector_repr());
      input_para_.target_velocity = toStd<7>(Vector7d::Zero());
      setInputLimits(input_para_, new_waypoint);

      target_robot_pose_ = new_target_robot_pose;
    }

    static inline std::array<double, 7> vec_cart_rot_elbow(double cart, double rot, double elbow) {
      return {cart, cart, cart, rot, rot, rot, elbow};
    }

    std::tuple<std::array<double, 7>, std::array<double, 7>, std::array<double, 7>>
    getInputLimits(const Waypoint &waypoint) const {
      constexpr double translation_factor{0.4};
      constexpr double derivative_factor{0.4};
      const Robot* robot = this->robot();

      if (waypoint.max_dynamics || params_.max_dynamics) {
        auto max_velocity = vec_cart_rot_elbow(
            0.8 * translation_factor * robot->max_translation_velocity,
            0.8 * robot->max_rotation_velocity,
            0.8 * robot->max_elbow_velocity
        );
        auto max_acceleration = vec_cart_rot_elbow(
            0.8 * translation_factor * derivative_factor * robot->max_translation_acceleration,
            0.8 * derivative_factor * robot->max_rotation_acceleration,
            0.8 * derivative_factor * robot->max_elbow_acceleration
        );
        auto max_jerk = vec_cart_rot_elbow(
            0.8 * translation_factor * std::pow(derivative_factor, 2) * robot->max_translation_jerk,
            0.8 * std::pow(derivative_factor, 2) * robot->max_rotation_jerk,
            0.8 * std::pow(derivative_factor, 2) * robot->max_elbow_jerk
        );
        return {max_velocity, max_acceleration, max_jerk};
      }

      auto max_velocity = vec_cart_rot_elbow(
          translation_factor * waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel *
          robot->max_translation_velocity,
          waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel * robot->max_rotation_velocity,
          waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel * robot->max_elbow_velocity
      );
      auto max_acceleration = vec_cart_rot_elbow(
          translation_factor * derivative_factor * params_.acceleration_rel * robot->acceleration_rel *
          robot->max_translation_acceleration,
          derivative_factor * params_.acceleration_rel * robot->acceleration_rel * robot->max_rotation_acceleration,
          derivative_factor * params_.acceleration_rel * robot->acceleration_rel * robot->max_elbow_acceleration
      );
      auto max_jerk = vec_cart_rot_elbow(
          translation_factor * std::pow(derivative_factor, 2) * params_.jerk_rel * robot->jerk_rel *
          robot->max_translation_jerk,
          std::pow(derivative_factor, 2) * params_.jerk_rel * robot->jerk_rel * robot->max_rotation_jerk,
          std::pow(derivative_factor, 2) * params_.jerk_rel * robot->jerk_rel * robot->max_elbow_jerk
      );
      return {max_velocity, max_acceleration, max_jerk};
    }

    void setInputLimits(ruckig::InputParameter<7> &input_parameters, const Waypoint &waypoint) const {
      std::tie(input_parameters.max_velocity, input_parameters.max_acceleration,
               input_parameters.max_jerk) = getInputLimits(waypoint);

      if (!(waypoint.max_dynamics || params_.max_dynamics) && waypoint.minimum_time.has_value()) {
        input_parameters.minimum_duration = waypoint.minimum_time.value();
      }

      if (waypoint.max_dynamics) {
        input_parameters.synchronization = ruckig::Synchronization::TimeIfNecessary;
      } else {
        input_parameters.synchronization = ruckig::Synchronization::Time;
      }
    }
  };


  class LinearMotion : public WaypointMotion {
    explicit LinearMotion(const RobotPose &target, bool relative = false, double velocity_rel = 1.0)
        : WaypointMotion{
        {
            {
                .robot_pose = target,
                .reference_type=relative ? Waypoint::ReferenceType::Relative
                                         : Waypoint::ReferenceType::Absolute,
                .velocity_rel = velocity_rel
            }
        }} {}
  };


  class StopMotion : public WaypointMotion {
    explicit StopMotion() : WaypointMotion{
        {{.reference_type=Waypoint::ReferenceType::Relative, .max_dynamics = true}}} {}
  };


  class PositionHold : public WaypointMotion {
    explicit PositionHold(double duration) : WaypointMotion{
        {{.reference_type=Waypoint::ReferenceType::Relative, .minimum_time = duration}}} {}
  };

} // namespace franky
