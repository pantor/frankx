#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <franky/types.hpp>
#include <franky/robot/motion_data.hpp>
#include <franky/robot/robot_state.hpp>
#include <franky/motion/motion_waypoint.hpp>
#include <ruckig/ruckig.hpp>


namespace franky {

  template<class RobotType>
  class WaypointMotionGenerator : public MotionGenerator {
    ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator{RobotType::control_rate};
    ruckig::InputParameter<RobotType::degrees_of_freedoms> input_para;
    ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_para;
    ruckig::Result result;

    std::vector<Waypoint>::iterator waypoint_iterator;
    bool waypoint_has_elbow{false};

    double time{0.0};
    RobotType *robot;

    Affine frame;
    MotionData &data;
    franka::RobotState asynchronous_state;

    constexpr static size_t cooldown_iterations{5};
    size_t current_cooldown_iteration{0};

    explicit WaypointMotionGenerator(RobotType *robot, const Affine &frame, WaypointMotion &motion, MotionData &data)
        : robot(robot), frame(frame), initial_motion_(motion), data(data) {}

    void init(const franka::RobotState &robot_state, franka::Duration period) {
      franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
      RobotPose robot_pose(initial_cartesian_pose);
      ref_frame_ = Affine();

      RobotPose zero_pose(Affine::Identity(), robot_pose.elbow_position().value());
      auto initial_velocity = Vector<6>::Map(robot_state.O_dP_EE_c.data());
      Vector7d initial_velocity_with_elbow = (Vector7d() << initial_velocity, robot_state.delbow_c[0]).finished();
      input_para.current_position = toStd(robot_pose.vector_repr());
      input_para.current_velocity = toStd(initial_velocity_with_elbow);
      input_para.current_acceleration = toStd(Vector7d::Zero());
      waypoint_has_elbow = true;

      setNewMotion(robot_state, initial_motion_);
    }

    franka::CartesianPose operator()(const franka::RobotState &robot_state, franka::Duration period) {
      time += period.toSec();
      if (time == 0.0) {
        init(robot_state, period);
      }

      asynchronous_state = franka::RobotState(robot_state);

      for (auto &reaction: data.reactions) {
        if (reaction.has_fired) {
          continue;  // Skip reactions that already fired
        }

        if (reaction.condition(MotionGenerator::convertState(robot_state), time)) {
          std::cout << "[franky] reaction fired." << std::endl;
          reaction.has_fired = true;

          if (reaction.waypoint_action.has_value() || reaction.waypoint_motion.has_value()) {
            if (reaction.waypoint_action.has_value()) {
              setNewMotion(reaction.waypoint_action.value()(MotionGenerator::convertState(robot_state), time));
            } else {
              setNewMotion(*(reaction.waypoint_motion.value()));
            }
          } else {
            robot->stop();
            return franka::MotionFinished(
                (ref_frame_ * RobotPose(input_para.current_position, !waypoint_has_elbow)).as_franka_pose());
          }
        }
      }

#ifdef WITH_PYTHON
      // Makes this motion generator stop if a KeyboardInterrupt is raised
      if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
        robot->stop();
      }
#endif

      const int steps = std::max<int>(period.toMSec(), 1);
      for (int i = 0; i < steps; i++) {
        result = trajectory_generator.update(input_para, output_para);

        if (initial_motion_.reload || result == ruckig::Result::Finished) {
          bool has_new_waypoint = false;
          bool done = false;

          if (waypoint_iterator != current_motion_.waypoints.end()) {
            ++waypoint_iterator;
            has_new_waypoint = (waypoint_iterator != current_motion_.waypoints.end());
          }

          while (!has_new_waypoint && !done) {
            if (current_motion_.reload) {
              waypoint_iterator = current_motion_.waypoints.begin();
              current_motion_.reload = false;
            } else if (initial_motion_.reload) {
              current_motion_ = initial_motion_;
              waypoint_iterator = current_motion_.waypoints.begin();
              initial_motion_.reload = false;
            } else {
              done = true;
            }
            has_new_waypoint = (waypoint_iterator != current_motion_.waypoints.end());
          }

          if (!has_new_waypoint && initial_motion_.return_when_finished) {
            auto output_pose = (
                ref_frame_ * RobotPose(input_para.current_position, !waypoint_has_elbow)).as_franka_pose();
            // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
            if (current_cooldown_iteration < cooldown_iterations) {
              current_cooldown_iteration += 1;
              return output_pose;
            }
            return franka::MotionFinished(output_pose);
          }

          if (has_new_waypoint) {
            setNewWaypoint(robot_state, *waypoint_iterator);
          }

        } else if (result == ruckig::Result::Error) {
          std::cout << "[franky robot] Invalid inputs:" << std::endl;
          return franka::MotionFinished(
              (ref_frame_ * RobotPose(input_para.current_position, !waypoint_has_elbow)).as_franka_pose());
        }

        output_para.pass_to_input(input_para);
      }

      return (ref_frame_ * RobotPose(input_para.current_position, !waypoint_has_elbow)).as_franka_pose();
    }

  private:
    WaypointMotion &initial_motion_;
    WaypointMotion current_motion_;
    RobotPose target_robot_pose_;

    Affine ref_frame_;

    void setNewMotion(const franka::RobotState &robot_state, const WaypointMotion &new_motion) {
      target_robot_pose_ = ref_frame_ * RobotPose(input_para.current_position, !waypoint_has_elbow);
      current_motion_ = new_motion;
      waypoint_iterator = current_motion_.waypoints.begin();
      setNewWaypoint(robot_state, *waypoint_iterator);
    }

    void setNewWaypoint(const franka::RobotState &robot_state, const Waypoint &new_waypoint) {
      // We first convert the current state into the frame of the current pose
      RobotPose current_pose = RobotPose(input_para.current_position, !waypoint_has_elbow);
      Affine current_pose_ref_frame = ref_frame_.inverse() * current_pose.end_effector_pose();
      ref_frame_ = current_pose.end_effector_pose();
      auto rot = current_pose_ref_frame.rotation().transpose();

      Vector<7> current_velocity = input_para.current_velocity;
      auto linear_vel_ref_frame = rot * current_velocity.head<3>();
      auto angular_vel_ref_frame = rot * current_velocity.segment<3>(3);

      Vector<7> current_acc = input_para.current_velocity;
      auto linear_acc_ref_frame = rot * current_acc.head<3>();
      auto angular_acc_ref_frame = rot * current_acc.segment<3>(3);

      RobotPose zero_pose(Affine::Identity(), current_pose.elbow_position());
      Vector7d current_velocity_ref_frame =
          (Vector7d() << linear_vel_ref_frame, angular_vel_ref_frame, current_velocity[6]).finished();
      Vector7d current_acc_ref_frame =
          (Vector7d() << linear_acc_ref_frame, angular_acc_ref_frame, current_acc[6]).finished();
      input_para.current_position = toStd(zero_pose.vector_repr());
      input_para.current_velocity = toStd(current_velocity_ref_frame);
      input_para.current_acceleration = toStd(current_acc_ref_frame);

      waypoint_has_elbow = new_waypoint.robot_pose.elbow_position().has_value();

      auto prev_target_robot_pose = target_robot_pose_;
      if (!target_robot_pose_.elbow_position().has_value()) {
        prev_target_robot_pose = prev_target_robot_pose.with_elbow_position(robot_state.elbow[0]);
      }
      auto new_target_robot_pose = new_waypoint.getTargetRobotPose(prev_target_robot_pose) * frame.inverse();
      auto new_target_robot_pose_ref_frame = ref_frame_.inverse() * new_target_robot_pose;
      input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
      input_para.target_position = toStd(new_target_robot_pose_ref_frame.vector_repr());
      input_para.target_velocity = toStd(Vector7d::Zero());
      setInputLimits(input_para, robot, new_waypoint, data);

      target_robot_pose_ = new_target_robot_pose;
    }
  };
} // namespace franky
