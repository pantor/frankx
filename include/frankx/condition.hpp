#pragma once

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

  explicit Condition(std::function<bool(const franka::RobotState&, double)> callback): callback(callback) { }
  explicit Condition(std::function<bool(const franka::RobotState&, double)> callback, std::shared_ptr<WaypointMotion> action): callback(callback), has_action(true), action(action) { }
  Condition(Measure measure, Comparison comparison, double value) {
    setCallback(measure, comparison, value);
  }
  Condition(Measure measure, Comparison comparison, double value, std::shared_ptr<WaypointMotion> action): has_action(true), action(action) {
    setCallback(measure, comparison, value);
  }

private:
  void setCallback(Measure measure, Comparison comparison, double value) {
    switch (measure) {
      case Measure::ForceZ: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = robot_state.O_F_ext_hat_K[2];
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = robot_state.O_F_ext_hat_K[2];
            return (force > value);
          };
        }
      } break;
      case Measure::ForceXYNorm: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
            return (force > value);
          };
        }
      } break;
      case Measure::ForceXYZNorm: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state, double time) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
            return (force > value);
          };
        }
      } break;
      case Measure::Time: {
        callback = [value](const franka::RobotState& robot_state, double time) {
          return (time > value);
        };
      }
    }
  }
};

} // namespace frankx