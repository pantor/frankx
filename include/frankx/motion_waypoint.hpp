#pragma once

#include <frankx/waypoint.hpp>


namespace frankx {

struct WaypointMotion {
    std::vector<Waypoint> waypoints;

    explicit WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}
};


struct LinearMotion: public WaypointMotion {
    explicit LinearMotion(const Affine& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow) }) { }
};


struct LinearRelativeMotion: public WaypointMotion {
    explicit LinearRelativeMotion(const Affine& affine): WaypointMotion({ Waypoint(affine, 0.0, Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativeMotion(const Affine& affine, double elbow): WaypointMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative) }) { }
};


struct PositionHold: public WaypointMotion {
    explicit PositionHold(double duration): WaypointMotion({ Waypoint(duration) }) { }
};

} // namespace frankx