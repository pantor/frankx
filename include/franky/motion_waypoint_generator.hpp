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
  struct WaypointMotionGenerator : public MotionGenerator {
    ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator{RobotType::control_rate};
    ruckig::InputParameter<RobotType::degrees_of_freedoms> input_para;
    ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_para;
    ruckig::Result result;

    WaypointMotion current_motion;
    std::vector<Waypoint>::iterator waypoint_iterator;
    bool waypoint_has_elbow{false};

    Affine old_affine;
    double old_elbow;
    double time{0.0};
    RobotType *robot;

    Affine frame;
    WaypointMotion &motion;
    MotionData &data;
    franka::RobotState asynchronous_state;

    Affine initial_pose;

    bool set_target_at_zero_time{true};

    const size_t cooldown_iterations{5};
    size_t current_cooldown_iteration{0};

    explicit WaypointMotionGenerator(RobotType *robot, const Affine &frame, WaypointMotion &motion, MotionData &data)
        : robot(robot), frame(frame), motion(motion), current_motion(motion), data(data) {}

    void reset() {
      time = 0.0;
      current_cooldown_iteration = 0;
      set_target_at_zero_time = false;
    }

    void init(const franka::RobotState &robot_state, franka::Duration period) {
      franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
      initial_pose = Eigen::Matrix4d::Map(initial_cartesian_pose.O_T_EE.data());

      Vector7d initial_vector_initial_pose_frame = (Vector7d()
          << 0, 0, 0, 0, 0, 0, initial_cartesian_pose.elbow[0]).finished();
      auto linear_vel_base_frame = (Eigen::Vector3d()
          << robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2]).finished();
      auto angular_vel_base_frame = (Eigen::Vector3d()
          << robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4]).finished();
      auto linear_vel_initial_pose_frame = initial_pose.rotation().transpose() * linear_vel_base_frame;
      auto angular_vel_initial_pose_frame = initial_pose.rotation().transpose() * angular_vel_base_frame;

      Vector7d initial_velocity_initial_pose_frame = (Vector7d()
          << linear_vel_initial_pose_frame, angular_vel_initial_pose_frame, robot_state.delbow_c[0]).finished();

      old_affine = initial_pose;
      old_elbow = initial_vector_initial_pose_frame(6);

      input_para.current_position = toStd(initial_vector_initial_pose_frame);
      input_para.current_velocity = toStd(initial_velocity_initial_pose_frame);
      input_para.current_acceleration = toStd(Vector7d::Zero());

      input_para.enabled = MotionGenerator::VectorCartRotElbow(true, true, true);
      setInputLimits(input_para, robot, data);

      if (set_target_at_zero_time) {
        waypoint_iterator = current_motion.waypoints.begin();

        const auto current_waypoint = *waypoint_iterator;
        waypoint_has_elbow = current_waypoint.elbow.has_value();
        auto target_position_vector = current_waypoint.getTargetVector(
            frame, old_affine, old_elbow, initial_pose.inverse());

        input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
        input_para.target_position = toStd(target_position_vector);
        input_para.target_velocity = toStd(Vector7d::Zero());
        setInputLimits(input_para, robot, current_waypoint, data);

        old_affine = current_waypoint.getTargetAffine(frame, old_affine);
        old_elbow = target_position_vector(6);
      }
    }

    franka::CartesianPose operator()(const franka::RobotState &robot_state, franka::Duration period) {
      time += period.toSec();
      if (time == 0.0) {
        init(robot_state, period);
      }

      asynchronous_state = franka::RobotState(robot_state);

      for (auto &reaction: data.reactions) {
        if (reaction.has_fired) {
          continue;
        }

        if (reaction.condition(MotionGenerator::convertState(robot_state), time)) {
          std::cout << "[franky] reaction fired." << std::endl;
          reaction.has_fired = true;

          bool new_motion = false;

          if (reaction.waypoint_action.has_value()) {
            new_motion = true;
            current_motion = reaction.waypoint_action.value()(MotionGenerator::convertState(robot_state), time);
          } else if (reaction.waypoint_motion.has_value()) {
            new_motion = true;
            current_motion = *(reaction.waypoint_motion.value());
          } else {
            robot->stop();
          }

          if (new_motion) {
            waypoint_iterator = current_motion.waypoints.begin();

            franka::CartesianPose current_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Affine current_pose;
            current_pose.matrix() = Eigen::Matrix4d::Map(current_cartesian_pose.O_T_EE.data());
            auto current_vector = RobotPose(current_pose, current_cartesian_pose.elbow[0]).vector_repr();
            old_affine = current_pose;
            old_elbow = current_cartesian_pose.elbow[0];

            const auto current_waypoint = *waypoint_iterator;
            waypoint_has_elbow = current_waypoint.elbow.has_value();
            auto target_position_vector = current_waypoint.getTargetVector(
                Affine::Identity(), old_affine, old_elbow, initial_pose.inverse());

            input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
            input_para.target_position = toStd(target_position_vector);
            input_para.target_velocity = toStd(Vector7d::Zero());
            setInputLimits(input_para, robot, current_waypoint, data);

            old_affine = current_waypoint.getTargetAffine(Affine::Identity(), old_affine);
            old_elbow = target_position_vector(6);
          } else {
            return franka::MotionFinished(MotionGenerator::CartesianPose(
                input_para.current_position, waypoint_has_elbow, initial_pose));
          }
        }
      }

#ifdef WITH_PYTHON
      if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
        robot->stop();
      }
#endif

      const int steps = std::max<int>(period.toMSec(), 1);
      for (int i = 0; i < steps; i++) {
        result = trajectory_generator.update(input_para, output_para);

        if (motion.reload || result == ruckig::Result::Finished) {
          bool has_new_waypoint{false};

          if (waypoint_iterator != current_motion.waypoints.end()) {
            ++waypoint_iterator;
            has_new_waypoint = (waypoint_iterator != current_motion.waypoints.end());
          }

          if (motion.return_when_finished && waypoint_iterator >= current_motion.waypoints.end()) {
            // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
            if (current_cooldown_iteration < cooldown_iterations) {
              current_cooldown_iteration += 1;
              return MotionGenerator::CartesianPose(
                  input_para.target_position, waypoint_has_elbow, initial_pose);
            }
            return franka::MotionFinished(MotionGenerator::CartesianPose(
                input_para.target_position, waypoint_has_elbow, initial_pose));

          } else if (motion.reload) {
            current_motion = motion;
            waypoint_iterator = current_motion.waypoints.begin();
            motion.reload = false;
            current_motion.reload = false;
            has_new_waypoint = true;
          }

          if (has_new_waypoint) {
            const auto current_waypoint = *waypoint_iterator;
            waypoint_has_elbow = current_waypoint.elbow.has_value();
            auto target_position_vector = current_waypoint.getTargetVector(
                frame, old_affine, old_elbow, initial_pose.inverse());

            input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
            input_para.target_position = toStd(target_position_vector);
            input_para.target_velocity = toStd(Vector7d::Zero());
            setInputLimits<RobotType>(input_para, robot, current_waypoint, data);

            old_affine = current_waypoint.getTargetAffine(frame, old_affine);
            old_elbow = target_position_vector(6);
          }

        } else if (result == ruckig::Result::Error) {
          std::cout << "[franky robot] Invalid inputs:" << std::endl;
          return franka::MotionFinished(MotionGenerator::CartesianPose(
              input_para.current_position, waypoint_has_elbow, initial_pose));
        }

        output_para.pass_to_input(input_para);
      }

      return MotionGenerator::CartesianPose(output_para.new_position, waypoint_has_elbow, initial_pose);
    }
  };

} // namespace franky