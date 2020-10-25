#pragma once

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>

#include <franka/robot_state.h>


namespace frankx {

struct Condition {
    using CallbackType = std::function<bool(const franka::RobotState&, double)>;
    CallbackType callback;

    explicit Condition(CallbackType callback): callback(callback) { }

    Condition& operator&&(const Condition& rhs) {
        callback = [this, rhs](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) && rhs.callback(robot_state, time);
        };
        return *this;
    }

    Condition& operator||(const Condition& rhs) {
        callback = [this, rhs](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) || rhs.callback(robot_state, time);
        };
        return *this;
    }
};

class Measure {
    using MeasureCallback = std::function<double(const franka::RobotState&, double)>;

    MeasureCallback callback;
    explicit Measure(MeasureCallback callback): callback(callback) { }

public:
    static Measure ForceZ() {
        return Measure([](const franka::RobotState& robot_state, double time) {
            return robot_state.O_F_ext_hat_K[2];
        });
    }

    static Measure ForceXYNorm() {
        return Measure([](const franka::RobotState& robot_state, double time) {
            return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
        });
    }

    static Measure ForceXYZNorm() {
        return Measure([](const franka::RobotState& robot_state, double time) {
            return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
        });
    }

    static Measure Time() {
        return Measure([](const franka::RobotState& robot_state, double time) {
            return time;
        });
    }

    Condition operator==(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) == threshold;
        });
    }

    Condition operator!=(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) != threshold;
        });
    }

    Condition operator<(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) < threshold;
        });
    }

    Condition operator<=(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) <= threshold;
        });
    }

    Condition operator>(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) > threshold;
        });
    }

    Condition operator>=(double threshold) {
        return Condition([this, threshold](const franka::RobotState& robot_state, double time) {
            return callback(robot_state, time) >= threshold;
        });
    }
};


class WaypointMotion;


struct Reaction {
    using WaypointAction = std::function<WaypointMotion(const franka::RobotState&, double)>;
    std::optional<WaypointAction> waypoint_action;
    std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion;

    Condition condition;
    bool has_fired {false};

    explicit Reaction(Condition::CallbackType callback): condition(callback) { }
    explicit Reaction(Condition::CallbackType callback, std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion): condition(callback), waypoint_motion(waypoint_motion) { }
    explicit Reaction(Condition::CallbackType callback, std::optional<WaypointAction> waypoint_action): condition(callback), waypoint_action(waypoint_action) { }

    explicit Reaction(Condition condition): condition(condition) { }
    explicit Reaction(Condition condition, std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion): condition(condition), waypoint_motion(waypoint_motion) { }
    explicit Reaction(Condition condition, std::optional<WaypointAction> waypoint_action): condition(condition), waypoint_action(waypoint_action) { }
};

} // namespace frankx
