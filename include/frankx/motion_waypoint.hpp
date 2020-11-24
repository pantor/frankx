#pragma once

#include <atomic>
#include <optional>

#include <movex/affine.hpp>
#include <movex/waypoint.hpp>


namespace frankx {

struct WaypointMotion {
    bool reload {false};
    bool return_when_finished {true};

    std::vector<movex::Waypoint> waypoints;

    explicit WaypointMotion(const std::vector<movex::Waypoint>& waypoints): waypoints(waypoints) {}
    explicit WaypointMotion(const std::vector<movex::Waypoint>& waypoints, bool return_when_finished): waypoints(waypoints), return_when_finished(return_when_finished) {}

    void setNextWaypoint(const movex::Waypoint& waypoint) {
        waypoints = { waypoint };
        reload = true;
    }

    void setNextWaypoints(const std::vector<movex::Waypoint>& waypoints) {
        this->waypoints = waypoints;
        reload = true;
    }

    void finish() {
        return_when_finished = true;
        reload = true;
    }
};


struct LinearMotion: public WaypointMotion {
    explicit LinearMotion(const Affine& target): WaypointMotion({ movex::Waypoint(target) }) { }
    explicit LinearMotion(const Affine& target, double elbow): WaypointMotion({ movex::Waypoint(target, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
    explicit LinearRelativeMotion(const Affine& affine): WaypointMotion({ movex::Waypoint(affine, movex::Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativeMotion(const Affine& affine, double elbow): WaypointMotion({ movex::Waypoint(affine, elbow, movex::Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativeMotion(const Affine& affine, double elbow, double dynamic_rel): WaypointMotion({ movex::Waypoint(affine, elbow, movex::Waypoint::ReferenceType::Relative, dynamic_rel) }) { }
};


struct StopMotion: public WaypointMotion {
    explicit StopMotion(): WaypointMotion({ movex::Waypoint(Affine(), 0.0, movex::Waypoint::ReferenceType::Relative, true) }) { }
    explicit StopMotion(const Affine& affine): WaypointMotion({ movex::Waypoint(affine, movex::Waypoint::ReferenceType::Relative, true) }) { }
    explicit StopMotion(const Affine& affine, double elbow): WaypointMotion({ movex::Waypoint(affine, elbow, movex::Waypoint::ReferenceType::Relative, true) }) { }
};


struct PositionHold: public WaypointMotion {
    explicit PositionHold(double duration): WaypointMotion({ movex::Waypoint(duration) }) { }
};

} // namespace frankx
