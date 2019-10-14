#pragma once

#include <frankx/condition.hpp>
#include <frankx/waypoint.hpp>


struct WaypointMotion {
  std::vector<Waypoint> waypoints;
  std::vector<Condition> conditions;

  WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}

  void add_condition(Condition condition) {
    conditions.push_back(condition);
  }
};


struct LinearMotion: public WaypointMotion {
  LinearMotion(const Eigen::Affine3d& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
  LinearRelativeMotion(const Eigen::Affine3d& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::RELATIVE) }) { }
};