#pragma once

#include <Eigen/Core>


namespace movex {

using Vector7d = Eigen::Matrix<double, 7, 1>;

class Segment {
public:
    virtual double get_length() const = 0;
    virtual Vector7d q(double s) const = 0;
    virtual Vector7d pdq(double s) const = 0;
    virtual Vector7d pddq(double s) const = 0;
    virtual Vector7d pdddq(double s) const = 0;
};


class LineSegment: public Segment {
    Vector7d start, end;
    double length;

public:
    explicit LineSegment(const Vector7d& start, const Vector7d&end): start(start), end(end) {
        Vector7d diff = end - start;

        length = 0.0;
    }

    double get_length() const {
        return length;
    }

    Vector7d q(double s) const {
        return start + s / length * (end - start);
    }

    Vector7d pdq(double s) const {
        return (end - start) / length;
    }

    Vector7d pddq(double s) const {
        return Vector7d::Zero();
    }

    Vector7d pdddq(double s) const {
        return Vector7d::Zero();
    }
};


class CircleSegment: public Segment {
    Vector7d center, x, y;
    double length, radius;

    explicit CircleSegment(const Vector7d& start, const Vector7d& intersection, const Vector7d& end, double max_deviation) {
        const Vector7d start_direction = (intersection - start).normalized();
		const Vector7d end_direction = (end - intersection).normalized();

        const double start_distance = (start - intersection).norm();
		const double end_distance = (end - intersection).norm();

        double distance = std::min((start - intersection).norm(), (end - intersection).norm());
		const double angle = acos(start_direction.dot(end_direction));

		distance = std::min(distance, max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

		radius = distance / tan(0.5 * angle);
		length = angle * radius;

		center = intersection + (end_direction - start_direction).normalized() * radius / cos(0.5 * angle);
		x = (intersection - distance * start_direction - center).normalized();
		y = start_direction;
    }

    double get_length() const {
        return length;
    }

    Vector7d q(double s) const {
        const double angle = s / radius;
        return center + radius * (x * cos(angle) + y * sin(angle));
    }

    Vector7d pdq(double s) const {
        const double angle = s / radius;
        return -x * sin(angle) + y * cos(angle);
    }

    Vector7d pddq(double s) const {
        const double angle = s / radius;
        return -1.0 / radius * (x * cos(angle) + y * sin(angle));
    }

    Vector7d pdddq(double s) const {
        const double angle = s / radius;
        return -1.0 / std::pow(radius, 2) * (-x * sin(angle) + y * cos(angle));
    }
};


class SplineSegment: public Segment {

};


struct PathPoint {
    Vector7d point;
    double max_blend_distance;

    PathPoint(const Vector7d& point): point(point) { }

    void blend_into_next(double max_distance) {
        max_blend_distance = max_distance;
    }
};


class Path {
    std::vector<Segment> segments;

    double length;

    explicit Path() { }
    explicit Path(const std::vector<PathPoint> waypoints) { }
};

} // namespace movex
