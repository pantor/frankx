#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>


#include "franky/robot_pose.hpp"
#include "franky/motion/waypoint_motion.hpp"


namespace franky {
  class LinearMotion : public WaypointMotion {
  public:
    explicit LinearMotion(const RobotPose &target, bool relative = false, double velocity_rel = 1.0);
  };
} // namespace franky
