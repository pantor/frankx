#pragma once

#include <cmath>
#include <functional>
#include <memory>

#include <franka/robot_state.h>

#include <tl/optional.hpp>


namespace frankx {

enum class Measure {
  ForceZ,
  ForceXYNorm,
  ForceXYZNorm,
  Time,
};

enum class Comparison {
  Equal,
  NotEqual,
  Greater,
  Less,
  GreaterEqual,
  LessEqual,
};


class WaypointMotion;

struct Reaction {
  std::function<bool(const franka::RobotState&, double)> condition_callback;
  bool has_fired {false};

  tl::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action;
  tl::optional<std::shared_ptr<WaypointMotion>> motion;

  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback);
  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback, tl::optional<std::shared_ptr<WaypointMotion>> motion);
  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback, tl::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action);
  explicit Reaction(Measure measure, Comparison comparison, double value);
  explicit Reaction(Measure measure, Comparison comparison, double value, tl::optional<std::shared_ptr<WaypointMotion>> motion);
  explicit Reaction(Measure measure, Comparison comparison, double value, tl::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action);

private:
  template<class T>
  static inline bool compare(Comparison comparison, T a, T b) {
    switch (comparison) {
      default:
      case Comparison::Equal:
        return a == b;
      case Comparison::NotEqual:
        return a != b;
      case Comparison::Greater:
        return a > b;
      case Comparison::Less:
        return a < b;
      case Comparison::GreaterEqual:
        return a >= b;
      case Comparison::LessEqual:
        return a <= b;
    }
  }

  void setConditionCallback(Measure measure, Comparison comparison, double value);
};

} // namespace frankx