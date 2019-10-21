#include <frankx/reaction.hpp>


namespace frankx {

Reaction::Reaction(std::function<bool(const franka::RobotState&, double)> callback): condition_callback(callback) { }

Reaction::Reaction(std::function<bool(const franka::RobotState&, double)> callback, tl::optional<std::shared_ptr<WaypointMotion>> motion): condition_callback(callback), motion(motion) { }

Reaction::Reaction(std::function<bool(const franka::RobotState&, double)> callback, tl::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action): condition_callback(callback), action(action) { }

Reaction::Reaction(Measure measure, Comparison comparison, double value) {
    setConditionCallback(measure, comparison, value);
}

Reaction::Reaction(Measure measure, Comparison comparison, double value, tl::optional<std::shared_ptr<WaypointMotion>> motion): motion(motion) {
    setConditionCallback(measure, comparison, value);
}

Reaction::Reaction(Measure measure, Comparison comparison, double value, tl::optional<std::function<WaypointMotion(const franka::RobotState&, double)>> action): action(action) {
    setConditionCallback(measure, comparison, value);
}


void Reaction::setConditionCallback(Measure measure, Comparison comparison, double value) {
    switch (measure) {
        case Measure::ForceZ: {
            if (comparison == Comparison::Smaller) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = robot_state.O_F_ext_hat_K[2];
                    return (force < value);
                };
            } else if (comparison == Comparison::Greater) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = robot_state.O_F_ext_hat_K[2];
                    return (force > value);
                };
            }
        } break;
        case Measure::ForceXYNorm: {
            if (comparison == Comparison::Smaller) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
                    return (force < value);
                };
            } else if (comparison == Comparison::Greater) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
                    return (force > value);
                };
            }
        } break;
        case Measure::ForceXYZNorm: {
            if (comparison == Comparison::Smaller) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
                    return (force < value);
                };
            } else if (comparison == Comparison::Greater) {
                condition_callback = [value](const franka::RobotState& robot_state, double time) {
                    double force = std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) + std::pow(robot_state.O_F_ext_hat_K[2], 2);
                    return (force > value);
                };
            }
        } break;
        case Measure::Time: {
            condition_callback = [value](const franka::RobotState& robot_state, double time) {
                return (time > value);
            };
        }
    }
}

} // namespace frankx