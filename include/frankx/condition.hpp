#pragma once

#include <memory>


class WaypointMotion;


struct Condition {
  enum class Axis {
    ForceZ,
    ForceXYZNorm,
  } axis;

  enum class Comparison {
    Greater,
    Smaller,
  } comparison;

  double value;
  bool fired {false};

  std::shared_ptr<WaypointMotion> action;

  explicit Condition(Axis axis, Comparison comparison, double value, std::shared_ptr<WaypointMotion> action): axis(axis), comparison(comparison), value(value), action(action) { }
};