#pragma once

#include <iostream>

#include <movex/robot/robot_state.hpp>


namespace movex {

class Condition {
    using RobotState_ = RobotState<7>;
public:
    using CallbackType = std::function<bool(const RobotState_&, double)>;

    explicit Condition(CallbackType callback): callback(callback) { }

    Condition& operator&&(const Condition& rhs) {
        callback = [this, rhs](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) && rhs.callback(robot_state, time);
        };
        return *this;
    }

    Condition& operator||(const Condition& rhs) {
        callback = [this, rhs](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) || rhs.callback(robot_state, time);
        };
        return *this;
    }

    //! Check if the condition is fulfilled
    bool operator()(const RobotState_& robot_state, double time) {
        return callback(robot_state, time);
    }

private:
    CallbackType callback;
};


class Measure {
    using RobotState_ = RobotState<7>;
    using MeasureCallback = std::function<double(const RobotState_&, double)>;

    MeasureCallback callback;
    explicit Measure(MeasureCallback callback): callback(callback) { }

public:
    static Measure ForceX() {
        return Measure([](const RobotState_& robot_state, double time) {
            return robot_state.O_F_ext_hat_K[0];
        });
    }

    static Measure ForceY() {
        return Measure([](const RobotState_& robot_state, double time) {
            return robot_state.O_F_ext_hat_K[1];
        });
    }

    static Measure ForceZ() {
        return Measure([](const RobotState_& robot_state, double time) {
            return robot_state.O_F_ext_hat_K[2];
        });
    }

    static Measure ForceXYNorm() {
        return Measure([](const RobotState_& robot_state, double time) {
            return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
        });
    }

    static Measure ForceXYZNorm() {
        return Measure([](const RobotState_& robot_state, double time) {
            return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
        });
    }

    static Measure Time() {
        return Measure([](const RobotState_& robot_state, double time) {
            return time;
        });
    }

    Condition operator==(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) == threshold;
        });
    }

    Condition operator!=(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) != threshold;
        });
    }

    Condition operator<(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) < threshold;
        });
    }

    Condition operator<=(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) <= threshold;
        });
    }

    Condition operator>(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) > threshold;
        });
    }

    Condition operator>=(double threshold) {
        return Condition([*this, threshold](const RobotState_& robot_state, double time) {
            return callback(robot_state, time) >= threshold;
        });
    }
};

} // namespace movex
