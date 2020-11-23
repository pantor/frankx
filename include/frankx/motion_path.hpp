#pragma once

#include <movex/path/path.hpp>


#include <Eigen/Core>


namespace frankx {

struct PathMotion {
    const std::vector<Affine> waypoints;
    double blend_max_distance {0.0};

    explicit PathMotion(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0): waypoints(waypoints), blend_max_distance(blend_max_distance) { }
};

} // namespace frankx
