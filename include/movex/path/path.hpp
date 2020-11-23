#pragma once

#include <memory>

#include <Eigen/Core>

#include <movex/affine.hpp>
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

    void init_path_points(const std::vector<PathPoint>& waypoints) {
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

                double s_abs_max = std::min<double>({ left->get_length() / 2, right->get_length() / 2 });

                auto blend = std::make_shared<QuarticBlendSegment>(left->start, lm, right->start, rm, left->get_length(), waypoints[i].blend_max_distance, s_abs_max);
                double s_abs = blend->get_length() / 2;

                auto new_left = std::make_shared<LineSegment>(left->start, left->q(left->get_length() - s_abs));
                auto new_right = std::make_shared<LineSegment>(right->q(s_abs), right->end);

                cumulative_length += new_left->get_length();
                segments.emplace_back(new_left);
                cumulative_lengths.emplace_back(cumulative_length);

                cumulative_length += blend->get_length();
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

public:
    constexpr static size_t degrees_of_freedom {7};

    explicit Path(const std::vector<PathPoint>& waypoints) {
        init_path_points(waypoints);
    }

    explicit Path(const std::vector<Affine>& waypoints, double blend_max_distance = 0.0) {
        std::vector<PathPoint> converted(waypoints.size());
        for (size_t i = 0; i < waypoints.size(); i += 1) {
            converted[i] = PathPoint(waypoints[i].vector_with_elbow(0.0), blend_max_distance);
        }
        init_path_points(converted);
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

    Vector7d max_pddq() const {
        Vector7d result;
        for (size_t i = 0; i < 7; i += 1) {
            auto max_ptr = *std::max_element(segments.begin(), segments.end(), [&](std::shared_ptr<Segment> l, std::shared_ptr<Segment> r){
                return std::abs(l->max_pddq()(i)) < std::abs(r->max_pddq()(i));
            });
            result(i) = std::abs(max_ptr->max_pddq()(i));
        }
        return result;
    }

    Vector7d max_pdddq() const {
        Vector7d result;
        for (size_t i = 0; i < 7; i += 1) {
            auto max_ptr = *std::max_element(segments.begin(), segments.end(), [&](std::shared_ptr<Segment> l, std::shared_ptr<Segment> r){
                return std::abs(l->max_pdddq()(i)) < std::abs(r->max_pdddq()(i));
            });
            result(i) = std::abs(max_ptr->max_pdddq()(i));
        }
        return result;
    }
};

} // namespace movex
