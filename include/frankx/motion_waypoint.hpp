#pragma once

#include <atomic>

#include <frankx/waypoint.hpp>


namespace frankx {

struct WaypointMotion {
    bool reload {false};
    bool return_when_finished {true};

    std::vector<Waypoint> waypoints;

    explicit WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}
    explicit WaypointMotion(const std::vector<Waypoint>& waypoints, bool return_when_finished): waypoints(waypoints), return_when_finished(return_when_finished) {}

    void setNextWaypoint(const Waypoint& waypoint) {
        waypoints = { waypoint };
        reload = true;
    }

    void setNextWaypoints(const std::vector<Waypoint>& waypoints) {
        this->waypoints = waypoints;
        reload = true;
    }

    void finish() {
        return_when_finished = true;
        reload = true;
    }
};


struct LinearMotion: public WaypointMotion {
    explicit LinearMotion(const Affine& target): WaypointMotion({ Waypoint(target) }) { }
    explicit LinearMotion(const Affine& target, double elbow): WaypointMotion({ Waypoint(target, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
    explicit LinearRelativeMotion(const Affine& affine): WaypointMotion({ Waypoint(affine, Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativeMotion(const Affine& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativeMotion(const Affine& affine, double elbow, double dynamic_rel): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative, dynamic_rel) }) { }
};


struct StopMotion: public WaypointMotion {
    explicit StopMotion(): WaypointMotion({ Waypoint(Affine(), 0.0, Waypoint::ReferenceType::Relative, true) }) { }
    explicit StopMotion(const Affine& affine): WaypointMotion({ Waypoint(affine, Waypoint::ReferenceType::Relative, true) }) { }
    explicit StopMotion(const Affine& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative, true) }) { }
};


struct PositionHold: public WaypointMotion {
    explicit PositionHold(double duration): WaypointMotion({ Waypoint(duration) }) { }
};

} // namespace frankx
