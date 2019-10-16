#pragma once

#include <cmath>
#include <functional>
#include <memory>

#include <franka/robot_state.h>


namespace frankx {

class WaypointMotion;

struct Condition {
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

  std::function<bool(const franka::RobotState&, double)> callback;
  bool has_fired {false};

  bool has_action {false};
  std::shared_ptr<WaypointMotion> action;

  explicit Condition(std::function<bool(const franka::RobotState&, double)> callback);
  explicit Condition(std::function<bool(const franka::RobotState&, double)> callback, std::shared_ptr<WaypointMotion> action);
  Condition(Measure measure, Comparison comparison, double value);
  Condition(Measure measure, Comparison comparison, double value, std::shared_ptr<WaypointMotion> action);

private:
  void setCallback(Measure measure, Comparison comparison, double value);
};

} // namespace frankx