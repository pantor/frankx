#pragma once

#include <memory>

#include <franka/robot_state.h>


namespace frankx {


class WaypointMotion;


struct Condition {
  enum class Axis {
    ForceZ,
    ForceXYNorm,
    ForceXYZNorm,
  };

  enum class Comparison {
    Greater,
    Smaller,
  };

  std::function<bool(const franka::RobotState&)> callback;
  bool has_fired {false};

  bool has_action {false};
  std::shared_ptr<WaypointMotion> action;

  explicit Condition(std::function<bool(const franka::RobotState&)> callback): callback(callback) { }
  explicit Condition(std::function<bool(const franka::RobotState&)> callback, std::shared_ptr<WaypointMotion> action): callback(callback), has_action(true), action(action) { }
  explicit Condition(Axis axis, Comparison comparison, double value) {
    setCallback(axis, comparison, value);
  }
  explicit Condition(Axis axis, Comparison comparison, double value, std::shared_ptr<WaypointMotion> action): has_action(true), action(action) {
    setCallback(axis, comparison, value);
  }

private:
  void setCallback(Axis axis, Comparison comparison, double value) {
    switch (axis) {
      case Axis::ForceZ: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = robot_state.O_F_ext_hat_K[2];
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = robot_state.O_F_ext_hat_K[2];
            return (force > value);
          };
        }
      } break;
      case Axis::ForceXYNorm: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
            return (force > value);
          };
        }
      } break;
      case Axis::ForceXYZNorm: {
        if (comparison == Comparison::Smaller) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
            return (force < value);
          };
        } else if (comparison == Comparison::Greater) {
          callback = [value](const franka::RobotState& robot_state) {
            double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
            return (force > value);
          };
        }
      } break;
    }
  }
};

} // namespace frankx