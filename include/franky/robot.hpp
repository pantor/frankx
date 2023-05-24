#pragma once

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "franky/types.hpp"
#include "franky/robot_pose.hpp"
#include "franky/kinematics.hpp"
#include "franky/motion/motion_generator.hpp"
#include "franky/scope_guard.hpp"


namespace franky {

  class Robot : public franka::Robot {
  public:
    struct Params {
      double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};

      // kCartesianImpedance wobbles -> setK?
      franka::ControllerMode controller_mode_{franka::ControllerMode::kJointImpedance};

      franka::RealtimeConfig realtime_config{franka::RealtimeConfig::kEnforce};
    };

    // Modified DH-parameters: alpha, d, a
    const KinematicChain<7> kinematics = KinematicChain<7>(
        {{
             {0.0, 0.333, 0.0},
             {-M_PI / 2, 0.0, 0.0},
             {M_PI / 2, 0.316, 0.0},
             {M_PI / 2, 0.0, 0.0825},
             {-M_PI / 2, 0.384, -0.0825},
             {M_PI / 2, 0.0, 0.0},
             {M_PI / 2, 0.0, 0.088}}},
        Affine().fromPositionOrientationScale(
            Eigen::Vector3d(0, 0, 0.107),
            Euler(M_PI / 4, 0, M_PI),
            Eigen::Matrix<double, 3, 1>::Ones())
    );

    // Cartesian constraints
    static constexpr double max_translation_velocity{1.7}; // [m/s]
    static constexpr double max_rotation_velocity{2.5}; // [rad/s]
    static constexpr double max_elbow_velocity{2.175}; // [rad/s]
    static constexpr double max_translation_acceleration{13.0}; // [m/s²]
    static constexpr double max_rotation_acceleration{25.0}; // [rad/s²]
    static constexpr double max_elbow_acceleration{10.0}; // [rad/s²]
    static constexpr double max_translation_jerk{6500.0}; // [m/s³]
    static constexpr double max_rotation_jerk{12500.0}; // [rad/s³]
    static constexpr double max_elbow_jerk{5000.0}; // [rad/s³]

    // Joint constraints
    static constexpr std::array<double, 7> max_joint_velocity{
        {2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}}; // [rad/s]
    static constexpr std::array<double, 7> max_joint_acceleration{
        {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0}}; // [rad/s²]
    static constexpr std::array<double, 7> max_joint_jerk{
        {7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0}}; // [rad/s^3]

    static constexpr size_t degrees_of_freedoms{7};
    static constexpr double control_rate{0.001}; // [s]

    //! Connects to a robot at the given FCI IP address.
    explicit Robot(const std::string &fci_ip);

    explicit Robot(const std::string &fci_ip, const Params &params);

    void setDynamicRel(double dynamic_rel);

    void setDynamicRel(double velocity_rel, double acceleration_rel, double jerk_rel);

    bool hasErrors();

    bool recoverFromErrors();

    Affine currentPose();

    Vector7d currentJointPositions();

    franka::RobotState state();

    inline double velocity_rel() const {
      return params_.velocity_rel;
    }

    inline double acceleration_rel() const {
      return params_.acceleration_rel;
    }

    inline double jerk_rel() const {
      return params_.jerk_rel;
    }

    // These helper functions are needed as the implicit template deduction does not work on subclasses of Motion
    inline void move(const std::shared_ptr<Motion<franka::CartesianPose>> &motion) {
      moveInternal<franka::CartesianPose>(motion, [this](const ControlFunc <franka::CartesianPose> &m) {
        control(m, params_.controller_mode_);
      });
    }

    inline void move(const std::shared_ptr<Motion<franka::CartesianVelocities>> &motion) {
      moveInternal<franka::CartesianVelocities>(motion, [this](const ControlFunc <franka::CartesianVelocities> &m) {
        control(m, params_.controller_mode_);
      });
    }

    inline void move(const std::shared_ptr<Motion<franka::JointPositions>> &motion) {
      moveInternal<franka::JointPositions>(motion, [this](const ControlFunc <franka::JointPositions> &m) {
        control(m, params_.controller_mode_);
      });
    }

    inline void move(const std::shared_ptr<Motion<franka::JointVelocities>> &motion) {
      moveInternal<franka::JointVelocities>(motion, [this](const ControlFunc <franka::JointVelocities> &m) {
        control(m, params_.controller_mode_);
      });
    }

    inline void move(const std::shared_ptr<Motion<franka::Torques>> &motion) {
      moveInternal<franka::Torques>(motion, [this](const ControlFunc <franka::Torques> &m) {
        control(m);
      });
    }

    static Affine forwardKinematics(const Vector7d &q);

    static Vector7d inverseKinematics(const Affine &target, const Vector7d &q0);

  private:
    template<typename ControlSignalType>
    using ControlFunc = std::function<ControlSignalType(const franka::RobotState &, franka::Duration)>;

    //! The robot's hostname / IP address
    std::string fci_ip_;
    Params params_;
    franka::RobotState current_state_;
    bool is_in_control_;
    std::mutex state_mutex_;

    template<typename ControlSignalType>
    inline void move(const std::shared_ptr<Motion<ControlSignalType>> &motion) {
      moveInternal<ControlSignalType>(motion, [this](const ControlFunc<ControlSignalType> &m) {
        control(m, params_.controller_mode_);
      });
    }

    template<typename ControlSignalType>
    void moveInternal(
        const std::shared_ptr<Motion<ControlSignalType>> &motion,
        const std::function<void(const ControlFunc<ControlSignalType> &)> &control_func) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_in_control_) {
        throw std::runtime_error("Robot is already in control.");
      }
      scope_guard is_in_control_guard([this]() { this->is_in_control_ = false; });
      is_in_control_ = true;
      MotionGenerator<ControlSignalType> motion_generator(this, motion);
      motion_generator.registerUpdateCallback(
          [this](const franka::RobotState &robot_state, franka::Duration duration, double time) {
            std::lock_guard<std::mutex> lock(this->state_mutex_);
            current_state_ = robot_state;
          });
      control_func(
          [&motion_generator](const franka::RobotState &rs, franka::Duration d) { return motion_generator(rs, d); });
    }
  };
} // namespace franky
