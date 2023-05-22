#pragma once

#include <Eigen/Core>

#include "franky/path/aggregated_path.hpp"
#include <franky/waypoint.hpp>


namespace franky {

/**
* A motion following a pre-defined path.
* Needs zero velocity and acceleration at start time.
*/
  struct PathMotion {
    std::vector<Waypoint> waypoints;

    explicit PathMotion(const std::vector<Waypoint> &waypoints) : waypoints(waypoints) {}
  };

  struct LinearPathMotion : public PathMotion {
    explicit LinearPathMotion(const RobotPose &target, bool relative = false)
        : PathMotion({
                         {.robot_pose = target, .reference_type=relative ? Waypoint::ReferenceType::Relative
                                                                         : Waypoint::ReferenceType::Absolute}}) {}
  };
} // namespace franky
