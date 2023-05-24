#include "franky/motion/linear_motion.hpp"

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>


#include "franky/robot_pose.hpp"
#include "franky/motion/waypoint_motion.hpp"


namespace franky {
  LinearMotion::LinearMotion(const RobotPose &target, bool relative, double velocity_rel)
      : WaypointMotion{
      {
          {
              .robot_pose = target,
              .reference_type=relative ? Waypoint::ReferenceType::Relative
                                       : Waypoint::ReferenceType::Absolute,
              .velocity_rel = velocity_rel
          }
      }} {}
} // namespace franky
