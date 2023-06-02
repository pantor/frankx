#include "franky/motion/linear_motion.hpp"

#include "franky/robot_pose.hpp"
#include "franky/motion/cartesian_waypoint_motion.hpp"

namespace franky {

LinearMotion::LinearMotion(
    const RobotPose &target,
    ReferenceType reference_type,
    const Affine& frame,
    double velocity_rel,
    double acceleration_rel,
    double jerk_rel,
    bool return_when_finished)
    : CartesianWaypointMotion(
    {
        CartesianWaypoint{
            {
                .velocity_rel = velocity_rel,
                .acceleration_rel = acceleration_rel,
                .jerk_rel = jerk_rel
            },
            target,
            reference_type
        }
    }, {
      {
        .return_when_finished = return_when_finished,
      },
      frame
    }) {}

}  // namespace franky
