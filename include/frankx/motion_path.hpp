#pragma once

#include <movex/path/path.hpp>


#include <Eigen/Core>


namespace frankx {

struct PathMotion {
    const std::vector<Affine> waypoints;

    //! The maximal distance of polynomial blending between two linear segments regarding their true path
    double blend_max_distance {0.0}; // [m]

    explicit PathMotion(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0): waypoints(waypoints), blend_max_distance(blend_max_distance) { }
};

} // namespace frankx
