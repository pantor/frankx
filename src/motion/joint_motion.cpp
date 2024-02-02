#include "franky/motion/joint_motion.hpp"

#include "franky/motion/joint_waypoint_motion.hpp"

namespace franky {

JointMotion::JointMotion(
    const Vector7d &target,
    ReferenceType reference_type,
    RelativeDynamicsFactor relative_dynamics_factor,
    bool return_when_finished)
    : JointWaypointMotion(
    {
        {
            .target = target,
            .reference_type = reference_type,
            .relative_dynamics_factor = relative_dynamics_factor
        }
    }, {
        Params{
            .return_when_finished = return_when_finished
        }
    }) {}

}  // namespace franky
