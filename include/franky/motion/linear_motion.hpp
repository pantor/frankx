#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/robot_pose.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"
#include "franky/motion/reference_type.hpp"

namespace franky {

class LinearMotion : public CartesianWaypointMotion {
 public:
  explicit LinearMotion(
      const RobotPose &target,
      ReferenceType reference_type = ReferenceType::Absolute,
      double velocity_rel = 1.0,
      double acceleration_rel = 1.0,
      double jerk_rel = 1.0,
      bool return_when_finished = true,
      const Affine &frame = Affine::Identity());
};

}  // namespace franky
