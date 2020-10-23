#pragma once

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>

#include <franka/robot_state.h>

// #include <frankx/motion.hpp>


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

struct Condition {
    using CallbackType = std::function<bool(const franka::RobotState&, double)>;

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

    template<class T>
    static CallbackType getConditionCallback(Measure measure, Comparison comparison, T value) {
        switch (measure) {
            case Measure::ForceZ:
                return [value, comparison](const franka::RobotState& robot_state, double time) {
                    double force = robot_state.O_F_ext_hat_K[2];
                    return compare(comparison, force, value);
                };
            case Measure::ForceXYNorm:
                return [value, comparison](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
                    return compare(comparison, force, value);
                };
            case Measure::ForceXYZNorm:
                return [value, comparison](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
                    return compare(comparison, force, value);
                };
            case Measure::Time:
                return [value, comparison](const franka::RobotState& robot_state, double time) {
                    return compare(comparison, time, value);
                };
        }
    }

    CallbackType callback;

    Condition(CallbackType callback): callback(callback) { }
    Condition(Measure measure, Comparison comparison, double value): callback(getConditionCallback(measure, comparison, value)) { }

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


class WaypointMotion;


struct Reaction {
    using WaypointAction = std::function<WaypointMotion(const franka::RobotState&, double)>;

    Condition condition;
    bool has_fired {false};

    std::optional<WaypointAction> waypoint_action;
    std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion;

    explicit Reaction(Condition::CallbackType callback): condition(callback) { }
    explicit Reaction(Condition::CallbackType callback, std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion): condition(callback), waypoint_motion(waypoint_motion) { }
    explicit Reaction(Condition::CallbackType callback, std::optional<WaypointAction> waypoint_action): condition(callback), waypoint_action(waypoint_action) { }

    explicit Reaction(Measure measure, Comparison comparison, double value): condition(measure, comparison, value) { }
    explicit Reaction(Measure measure, Comparison comparison, double value, std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion): condition(measure, comparison, value), waypoint_motion(waypoint_motion) { }
    explicit Reaction(Measure measure, Comparison comparison, double value, std::optional<WaypointAction> waypoint_action): condition(measure, comparison, value), waypoint_action(waypoint_action) { }

    explicit Reaction(Condition condition): condition(condition) { }
    explicit Reaction(Condition condition, std::optional<std::shared_ptr<WaypointMotion>> waypoint_motion): condition(condition), waypoint_motion(waypoint_motion) { }
    explicit Reaction(Condition condition, std::optional<WaypointAction> waypoint_action): condition(condition), waypoint_action(waypoint_action) { }
};

} // namespace frankx