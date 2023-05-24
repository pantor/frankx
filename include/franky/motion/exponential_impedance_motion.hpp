#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "franky/robot_pose.hpp"
#include "franky/motion/impedance_motion.hpp"


namespace franky {
  class ExponentialImpedanceMotion : public ImpedanceMotion {
  public:
    struct Params : public ImpedanceMotion::Params {
      double exponential_decay{0.005};
    };

    explicit ExponentialImpedanceMotion(const Affine &target);

    explicit ExponentialImpedanceMotion(const Affine &target, const Params& params);

  protected:
    std::tuple<Affine, bool>
    update(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

  private:
    Params params_;
  };
} // namespace franky
