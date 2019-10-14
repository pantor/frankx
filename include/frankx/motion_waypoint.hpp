#pragma once

#include <frankx/waypoint.hpp>


struct WaypointMotion {
  std::vector<Waypoint> waypoints;

  explicit WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}
};


struct LinearMotion: public WaypointMotion {
  explicit LinearMotion(const Eigen::Affine3d& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
  explicit LinearRelativeMotion(const Eigen::Affine3d& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative) }) { }
};