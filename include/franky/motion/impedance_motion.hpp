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
    explicit ImpedanceMotion(const Affine &target, const Params &params);

  protected:
    void initImpl(const franka::RobotState &robot_state, double time) override;

    franka::Torques
    nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

    inline Affine intermediate_target() const {
      return intermediate_target_;
    }

    inline Affine target() const {
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
} // namespace franky
