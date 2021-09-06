#pragma once

#include <iostream>
#include <memory>

#include <Eigen/Core>

#include <affx/affine.hpp>
#include <movex/waypoint.hpp>
#include <movex/path/segment.hpp>


namespace movex {

class Path {
    using Affine = affx::Affine;

    std::vector<double> cumulative_lengths;

    double length {0.0};

    void init_path_points(const std::vector<Waypoint>& waypoints);

public:
    constexpr static size_t degrees_of_freedom {7};

    std::vector<std::shared_ptr<Segment>> segments;
    size_t get_index(double s) const;
    std::tuple<std::shared_ptr<Segment>, double> get_local(double s) const;

    explicit Path() { }
    explicit Path(const std::vector<Waypoint>& waypoints);
    explicit Path(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0);

    double get_length() const;

    Vector7d q(double s) const;
    Vector7d q(double s, const Affine& frame) const;
    Vector7d pdq(double s) const;
    Vector7d pddq(double s) const;
    Vector7d pdddq(double s) const;

    Vector7d dq(double s, double ds) const;
    Vector7d ddq(double s, double ds, double dds) const;
    Vector7d dddq(double s, double ds, double dds, double ddds) const;

    Vector7d max_pddq() const;
    Vector7d max_pdddq() const;
};

} // namespace movex
