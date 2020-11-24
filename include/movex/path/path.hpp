#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <movex/affine.hpp>
#include <movex/waypoint.hpp>
#include <movex/path/segment.hpp>


namespace movex {

class Path {
    std::vector<std::shared_ptr<Segment>> segments;
    std::vector<double> cumulative_lengths;

    double length {0.0};

    std::tuple<std::shared_ptr<Segment>, double> get_local(double s) const;

    void init_path_points(const std::vector<Waypoint>& waypoints);

public:
    constexpr static size_t degrees_of_freedom {7};

    explicit Path(const std::vector<Waypoint>& waypoints);
    explicit Path(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0);

    double get_length() const;

    Vector7d q(double s) const;
    Vector7d pdq(double s) const;
    Vector7d pddq(double s) const;
    Vector7d pdddq(double s) const;

    Vector7d max_pddq() const;
    Vector7d max_pdddq() const;
};

} // namespace movex
