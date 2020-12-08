#pragma once

#include <cmath>
#include <iostream>
#include <functional>
#include <memory>
#include <optional>

#include <movex/robot/measure.hpp>


namespace movex {

class WaypointMotion;

struct Reaction {
    using WaypointAction = std::function<WaypointMotion(const RobotState<7>&, double)>;
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

} // namespace movex
