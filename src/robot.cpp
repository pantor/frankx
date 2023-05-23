#include <ruckig/ruckig.hpp>

#include "franky/robot.hpp"


namespace franky {
  using Affine = Eigen::Affine3d;

  //! Connects to a robot at the given FCI IP address.
  Robot::Robot(const std::string &fci_ip) : Robot(fci_ip, Params()) {}

  Robot::Robot(const std::string &fci_ip, const Params &params)
      : fci_ip_(fci_ip), params_(params), is_in_control_(false), franka::Robot(fci_ip, params.realtime_config) {}

  void Robot::setDynamicRel(double dynamic_rel) {
    setDynamicRel(dynamic_rel, dynamic_rel, dynamic_rel);
  }

  void Robot::setDynamicRel(double velocity_rel, double acceleration_rel, double jerk_rel) {
    params_.velocity_rel = velocity_rel;
    params_.acceleration_rel = acceleration_rel;
    params_.jerk_rel = jerk_rel;
  }

  bool Robot::hasErrors() {
    return bool(state().current_errors);
  }

  bool Robot::recoverFromErrors() {
    automaticErrorRecovery();
    return !hasErrors();
  }

  Affine Robot::currentPose() {
    return Affine(Eigen::Matrix4d::Map(state().O_T_EE.data()));
  }

  Vector7d Robot::currentJointPositions() {
    return Eigen::Map<const Vector7d>(state().q.data());
  }

  Affine Robot::forwardKinematics(const Vector7d &q) {
    return Kinematics::forward(q);
  }

  Vector7d Robot::inverseKinematics(const Affine &target, const Vector7d &q0) {
    Vector7d result;

    Eigen::Vector3d angles = Euler(target.rotation()).angles();
    Eigen::Vector3d angles_norm;
    angles_norm << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;

    if (angles_norm[1] > M_PI) {
      angles_norm[1] -= 2 * M_PI;
    }
    if (angles_norm[2] < -M_PI) {
      angles_norm[2] += 2 * M_PI;
    }

    if (angles.norm() < angles_norm.norm()) {
      angles_norm = angles;
    }

    Eigen::Matrix<double, 6, 1> x_target;
    x_target << target.translation(), angles;

    return Kinematics::inverse(x_target, q0);
  }

  franka::RobotState Robot::state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_in_control_) {
      current_state_ = readOnce();
    }
    return current_state_;
  }

  template<>
  void Robot::move<franka::Torques>(const std::shared_ptr<Motion<franka::Torques>> &motion) {
    moveInternal<franka::Torques>(motion, [this](const ControlFunc<franka::Torques> &m) {
      control(m);
    });
  }

//
//  bool Robot::move(const Affine &frame, WaypointMotion &motion, MotionData &data) {
//    WaypointMotionGenerator <Robot> mg{this, frame, motion, data};
//    mg.input_para.target_position[0] = 0.01;
//
//    asynchronous_state_ptr = &mg.asynchronous_state;
//
//    try {
//      control(mg, controller_mode);
////      control(stateful<franka::CartesianPose>(mg), controller_mode);
//    } catch (franka::Exception exception) {
//      auto errors = readOnce().last_motion_errors;
//      if (repeat_on_error
//        // && (errors.cartesian_motion_generator_joint_acceleration_discontinuity
//        // || errors.cartesian_motion_generator_joint_velocity_discontinuity
//        // || errors.cartesian_motion_generator_velocity_discontinuity
//        // || errors.cartesian_motion_generator_acceleration_discontinuity)
//          ) {
//        std::cout << "[franky robot] continue motion after exception: " << exception.what() << std::endl;
//        automaticErrorRecovery();
//
//        data.velocity_rel *= 0.5;
//        data.acceleration_rel *= 0.5;
//        data.jerk_rel *= 0.5;
//        mg.reset();
//
//        bool success{false};
//
//        try {
//          control(mg, controller_mode);
//          // control(stateful<franka::CartesianPose>(mg), controller_mode);
//          success = true;
//
//        } catch (franka::Exception exception) {
//          std::cout << exception.what() << std::endl;
//        }
//        data.velocity_rel *= 2;
//        data.acceleration_rel *= 2;
//        data.jerk_rel *= 2;
//
//        return success;
//
//      } else {
//        std::cout << exception.what() << std::endl;
//      }
//
//      return false;
//    }
//    return true;
//  }

} // namepace franky
