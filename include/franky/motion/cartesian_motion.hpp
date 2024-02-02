#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/robot_pose.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"
#include "franky/motion/reference_type.hpp"

namespace franky {

class CartesianMotion : public CartesianWaypointMotion {
 public:
  explicit CartesianMotion(
      const RobotPose &target,
      ReferenceType reference_type = ReferenceType::Absolute,
      const Affine &frame = Affine::Identity(),
      RelativeDynamicsFactor relative_dynamics_factor = 1.0,
      bool return_when_finished = true);
};

// Backwards compatibility
using LinearMotion = CartesianMotion;

}  // namespace franky
