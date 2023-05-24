#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/robot_pose.hpp"
#include "franky/motion/motion.hpp"


namespace franky {
  class ImpedanceMotion : public Motion<franka::Torques> {
  public:
  public:
    enum class TargetType {
      Absolute,
      Relative
    };

    struct Params {
      TargetType target_type{TargetType::Absolute};
      double translational_stiffness{2000};
      double rotational_stiffness{200};
      Eigen::Vector<double, 6> force_constraints;
      Eigen::Vector<bool, 6> force_constraints_active{Eigen::Vector<bool, 6>::Zero()};
    };

    ///
    /// \param frame
    /// \param translational_stiffness  in [10, 3000] N/m
    /// \param rotational_stiffness     in [1, 300] Nm/rad
    explicit ImpedanceMotion(const Affine &target, const Params &params)
        : target_(target), params_(params), Motion<franka::Torques>() {
      stiffness.setZero();
      stiffness.topLeftCorner(3, 3) << params.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      stiffness.bottomRightCorner(3, 3) << params.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      damping.setZero();
      damping.topLeftCorner(3, 3) << 2.0 * sqrt(params.translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
      damping.bottomRightCorner(3, 3) << 2.0 * sqrt(params.rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    }

  protected:
    void initImpl(const franka::RobotState &robot_state, double time) override {
      auto robot_pose = Affine(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      intermediate_target_ = robot_pose;
      if (params_.target_type == TargetType::Relative)
        absolute_target_ = robot_pose * target_;
      else
        absolute_target_ = target_;
      model_ = std::make_unique<franka::Model>(robot()->loadModel());
    }

    franka::Torques
    nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) override {
      std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
      std::array<double, 42> jacobian_array = model_->zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - intermediate_target_.translation();

      Eigen::Quaterniond quat(target().rotation());
      if (quat.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }

      Eigen::Quaterniond error_quaternion(orientation.inverse() * quat);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      error.tail(3) << -transform.linear() * error.tail(3);

      auto wrench_cartesian_default = -stiffness * error - damping * (jacobian * dq);
      auto wrench_cartesian = params_.force_constraints_active.select(
          params_.force_constraints, wrench_cartesian_default);

      auto tau_task = jacobian.transpose() * wrench_cartesian;
      auto tau_d = tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // Update target with target motion
      auto [intermediate_target, finish] = update(robot_state, time_step, time);
      intermediate_target_ = intermediate_target;

      auto output = franka::Torques(tau_d_array);
      if (finish)
        output = franka::MotionFinished(output);

      return output;
    }

    Affine intermediate_target() const {
      return intermediate_target_;
    }

    Affine target() const {
      return absolute_target_;
    }

    virtual std::tuple<Affine, bool>
    update(const franka::RobotState &robot_state, franka::Duration time_step, double time) = 0;

  private:
    Affine absolute_target_;
    Affine target_;
    Params params_;

    Eigen::Matrix<double, 6, 6> stiffness, damping;
    Affine intermediate_target_;

    std::unique_ptr<franka::Model> model_;
  };

  class ExponentialImpedanceMotion : public ImpedanceMotion {
  public:
    struct Params : public ImpedanceMotion::Params {
      double exponential_decay{0.005};
    };

    explicit ExponentialImpedanceMotion(const Affine &target)
        : ExponentialImpedanceMotion(target, Params()) {}

    explicit ExponentialImpedanceMotion(const Affine &target, Params params)
        : params_(params), ImpedanceMotion(target, params) {}

  protected:
    std::tuple<Affine, bool>
    update(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
      auto trans = params_.exponential_decay * target().translation() +
                   (1.0 - params_.exponential_decay) * intermediate_target().translation();
      auto rot = Eigen::Quaterniond(intermediate_target().rotation()).slerp(
          params_.exponential_decay, Eigen::Quaterniond(target().rotation()));
      return {Affine().fromPositionOrientationScale(trans, rot, Eigen::Vector3d::Ones()), false};
    }

  private:
    Params params_;
  };

  class LinearImpedanceMotion : public ImpedanceMotion {
  public:
    struct Params : public ImpedanceMotion::Params {
      bool return_when_finished{true};
      double finish_wait_factor{1.2}; // Wait a bit longer to stop
    };

    explicit LinearImpedanceMotion(const Affine &target, double duration)
        : LinearImpedanceMotion(target, duration, Params()) {}

    explicit LinearImpedanceMotion(const Affine &target, double duration, const Params &params)
        : duration_(duration), params_(params), ImpedanceMotion(target, params) {}

  protected:
    void initImpl(const franka::RobotState &robot_state, double time) {
      ImpedanceMotion::initImpl(robot_state, time);
      initial_pose_ = Affine(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      motion_init_time_ = time;
    }

    std::tuple<Affine, bool>
    update(const franka::RobotState &robot_state, franka::Duration time_step, double time) {
      double transition_parameter = (time - motion_init_time_) / duration_;
      Affine intermediate_goal;
      bool done;
      if (transition_parameter <= 1.0) {  // [ms] to [s]
        Eigen::Quaterniond q_start(initial_pose_.rotation());
        Eigen::Quaterniond q_end(target().rotation());
        auto init_trans = initial_pose_.translation();
        auto trans = init_trans + transition_parameter * (target().translation() - init_trans);
        auto rot = q_start.slerp(transition_parameter, q_end);
        intermediate_goal = Affine().fromPositionOrientationScale(trans, rot, Eigen::Vector3d::Ones());
        done = false;
      } else if (params_.return_when_finished && transition_parameter > params_.finish_wait_factor) {
        done = true;
        intermediate_goal = target();
      }
      return {intermediate_goal, done};
    }

  private:
    Affine initial_pose_;
    double motion_init_time_;
    double duration_;
    Params params_;
  };
} // namespace franky
