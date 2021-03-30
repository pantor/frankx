#pragma once

#include <Eigen/Core>

#include <movex/path/path.hpp>
#include <movex/waypoint.hpp>


namespace movex {

/**
* A motion following a pre-defined path.
* Needs zero velocity and acceleration at start time.
*/
struct PathMotion {
    using Affine = affx::Affine;

    std::vector<Waypoint> waypoints;

    explicit PathMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) { }
    explicit PathMotion(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0) {
        this->waypoints.resize(waypoints.size());
        for (size_t i = 0; i < waypoints.size(); i += 1) {
            this->waypoints[i] = Waypoint(waypoints[i], std::nullopt, blend_max_distance);
        }
    }
};


struct LinearPathMotion: public PathMotion {
    explicit LinearPathMotion(const Affine& target): PathMotion({ Waypoint(target) }) { }
    explicit LinearPathMotion(const Affine& target, double elbow): PathMotion({ Waypoint(target, elbow) }) { }
};


struct LinearRelativePathMotion: public PathMotion {
    explicit LinearRelativePathMotion(const Affine& affine): PathMotion({ Waypoint(affine, Waypoint::ReferenceType::Relative) }) { }
    explicit LinearRelativePathMotion(const Affine& affine, double elbow): PathMotion({ Waypoint(affine, elbow, Waypoint::ReferenceType::Relative) }) { }
};

} // namespace movex
