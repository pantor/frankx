#pragma once

#include <atomic>

#include <frankx/waypoint.hpp>


namespace frankx {

struct WaypointMotion {
    bool reload {false};

    std::vector<Waypoint> waypoints;

    explicit WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}

    void setNextWaypoint(const Waypoint& waypoint) {
        waypoints = { waypoint };
        reload = true;
    }

    void setNextWaypoints(const std::vector<Waypoint>& waypoints) {
        this->waypoints = waypoints;
        reload = true;
    }
};


struct LinearMotion: public WaypointMotion {
    explicit LinearMotion(const Affine& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
    explicit LinearRelativeMotion(const Affine& affine, double elbow = 0.0): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative) }) { }
};


struct PositionHold: public WaypointMotion {
    explicit PositionHold(double duration): WaypointMotion({ Waypoint(duration) }) { }
};

} // namespace frankx