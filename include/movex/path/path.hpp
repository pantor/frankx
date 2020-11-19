#pragma once

#include <memory>

#include <Eigen/Core>

#include <movex/path/segment.hpp>


namespace movex {

struct PathPoint {
    Vector7d point;
    double blend_max_distance {0.0};

    PathPoint() { }
    PathPoint(const Vector7d& point, double blend_max_distance = 0.0): point(point), blend_max_distance(blend_max_distance) { }
};


class Path {
    std::vector<std::shared_ptr<Segment>> segments;
    std::vector<double> cumulative_lengths;

    double length {0.0};

    std::tuple<std::shared_ptr<Segment>, double> get_local(double s) const {
        auto ptr = std::lower_bound(cumulative_lengths.begin(), cumulative_lengths.end(), s);
        size_t index = std::distance(cumulative_lengths.begin(), ptr);

        auto segment = segments[index];
        double s_local = (index == 0) ? s : s - cumulative_lengths[index - 1];
        return {segment, s_local};
    }

    inline double Power(double v, int e) {
        return std::pow(v, e);
    }

    std::tuple<QuinticPolynomialSegment, Vector7d> blend_polynomial(const Vector7d& lb, const Vector7d& lm, const Vector7d& rb, const Vector7d& rm, const Vector7d& sMid, const Vector7d& maxDiff) {
        Vector7d sAbs_ = (-8*(lb.array() + 2*maxDiff.array() - rb.array() + lm.array()*sMid.array()))/(3.*(lm - rm).array());
        Vector7d sAbsMin = Vector7d::Constant(sAbs_.minCoeff());

        Vector7d a = Vector7d::Zero();
        Vector7d b = (lm - rm).array() / (16.*sAbsMin.array().pow(3));
        Vector7d c = ((-lm + rm).array()*sMid.array())/(4.*sAbsMin.array().pow(3));
        Vector7d d = (-3*(lm - rm).array()*(sAbsMin.array().pow(2) - sMid.array().pow(2)))/(8.*sAbsMin.array().pow(3));
        Vector7d e = (lm.array()*(2*sAbsMin.array() - sMid.array())*(sAbsMin + sMid).array().pow(2) + rm.array()*(sAbsMin - sMid).array().pow(2)*(2*sAbsMin.array() + sMid.array()))/(4.*sAbsMin.array().pow(3));
        Vector7d f = lb.array() - ((lm - rm).array()*(sAbsMin - sMid).array().pow(3)*(3*sAbsMin.array() + sMid.array()))/(16.*sAbsMin.array().pow(3));

        auto blend = QuinticPolynomialSegment(a, b, c, d, e, f);
        return {blend, sAbsMin};
    }

public:
    constexpr static size_t degrees_of_freedom {7};

    explicit Path(const std::vector<PathPoint>& waypoints) {
        double cumulative_length {0.0};
        for (size_t i = 0; i < waypoints.size() - 1; i += 1) {
            auto segment = std::make_shared<LineSegment>(waypoints[i].point, waypoints[i+1].point);
            cumulative_length += segment->get_length();
            segments.emplace_back(segment);
            cumulative_lengths.emplace_back(cumulative_length);
        }
        length = cumulative_length;
    }

    static Path Linear(const std::vector<Vector7d>& waypoints, double blend_max_distance = 0.0) {
        std::vector<PathPoint> converted(waypoints.size());
        for (size_t i = 0; i < waypoints.size(); i += 1) {
            converted[i] = PathPoint(waypoints[i], blend_max_distance);
        }
        return Path(converted);
    }

    double get_length() const {
        return length;
    }

    Vector7d q(double s) const {
        auto [segment, s_local] = get_local(s);
        return segment->q(s_local);
    }

    Vector7d pdq(double s) const {
        auto [segment, s_local] = get_local(s);
        return segment->pdq(s_local);
    }

    Vector7d pddq(double s) const {
        auto [segment, s_local] = get_local(s);
        return segment->pddq(s_local);
    }

    Vector7d pdddq(double s) const {
        auto [segment, s_local] = get_local(s);
        return segment->pddq(s_local);
    }
};

} // namespace movex
