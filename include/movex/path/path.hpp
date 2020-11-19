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
        index = std::min(index, segments.size() - 1);

        auto segment = segments.at(index);
        double s_local = (index == 0) ? s : s - cumulative_lengths[index - 1];
        return {segment, s_local};
    }

    std::tuple<std::shared_ptr<QuinticSegment>, double> blend_polynomial(const Vector7d& lb, const Vector7d& lm, const Vector7d& rb, const Vector7d& rm, double s_mid, double max_diff) {
        Vector7d sAbs_ = ((-16*max_diff)/(3.*(lm - rm).array())).abs();
        double s_abs_min = sAbs_.minCoeff();

        Vector7d a = Vector7d::Zero();
        Vector7d b = (lm - rm).array() / (16.*std::pow(s_abs_min, 3));
        Vector7d c = (-lm + rm).array() / (4.*std::pow(s_abs_min, 2));
        Vector7d d = Vector7d::Zero();
        Vector7d e = lm;
        Vector7d f = lb.array() + lm.array()*(-s_abs_min + s_mid);

        return {std::make_shared<QuinticSegment>(a, b, c, d, e, f, 2*s_abs_min), s_abs_min};
    }

public:
    constexpr static size_t degrees_of_freedom {7};

    explicit Path(const std::vector<PathPoint>& waypoints) {
        std::vector<std::shared_ptr<LineSegment>> line_segments;

        for (size_t i = 0; i < waypoints.size() - 1; i += 1) {
            auto segment = std::make_shared<LineSegment>(waypoints[i].point, waypoints[i+1].point);
            line_segments.emplace_back(segment);
        }

        double cumulative_length {0.0};
        for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
            if (waypoints[i].blend_max_distance > 0.0) {
                auto& left = line_segments[i - 1];
                auto& right = line_segments[i];

                Vector7d lm = (left->end - left->start) / left->get_length();
                Vector7d rm = (right->end - right->start) / right->get_length();

                auto [blend, s_abs] = blend_polynomial(left->start, lm, right->start, rm, left->get_length(), waypoints[i].blend_max_distance);
                auto new_left = std::make_shared<LineSegment>(left->start, left->q(left->get_length() - s_abs));
                auto new_right = std::make_shared<LineSegment>(right->q(s_abs), right->end);

                cumulative_length += new_left->get_length();
                segments.emplace_back(new_left);
                cumulative_lengths.emplace_back(cumulative_length);

                cumulative_length += 2 * s_abs;
                segments.emplace_back(blend);
                cumulative_lengths.emplace_back(cumulative_length);

                right = new_right;

            } else {
                cumulative_length += line_segments[i - 1]->get_length();
                segments.emplace_back(line_segments[i - 1]);
                cumulative_lengths.emplace_back(cumulative_length);
            }
        }

        cumulative_length += line_segments.back()->get_length();
        segments.emplace_back(line_segments.back());
        cumulative_lengths.emplace_back(cumulative_length);
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
