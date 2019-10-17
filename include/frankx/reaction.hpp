#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>

#include <franka/robot_state.h>


namespace frankx {

enum class Measure {
  ForceZ,
  ForceXYNorm,
  ForceXYZNorm,
  Time,
};

enum class Comparison {
  Greater,
  Smaller,
};


class WaypointMotion;

struct Reaction {
  std::function<bool(const franka::RobotState&, double)> condition_callback;
  bool has_fired {false};

  std::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action;
  std::optional<std::shared_ptr<WaypointMotion>> motion;

  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback);
  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback, std::optional<std::shared_ptr<WaypointMotion>> motion);
  explicit Reaction(std::function<bool(const franka::RobotState&, double)> callback, std::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action);
  explicit Reaction(Measure measure, Comparison comparison, double value);
  explicit Reaction(Measure measure, Comparison comparison, double value, std::optional<std::shared_ptr<WaypointMotion>> motion);
  explicit Reaction(Measure measure, Comparison comparison, double value, std::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action);

private:
  void setConditionCallback(Measure measure, Comparison comparison, double value);
};

} // namespace frankx