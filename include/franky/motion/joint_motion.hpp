#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/robot_pose.hpp"
#include "franky/motion/joint_waypoint_motion.hpp"
#include "franky/motion/reference_type.hpp"

namespace franky {

class JointMotion : public JointWaypointMotion {
 public:
  explicit JointMotion(
      const Vector7d &target,
      ReferenceType reference_type,
      RelativeDynamicsFactor relative_dynamics_factor,
      bool return_when_finished);
};

}  // namespace franky
