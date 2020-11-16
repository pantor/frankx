#pragma once

#include <Eigen/Core>


namespace frankx {

class Segment {
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;

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

};


class SplineSegment: public Segment {

};


struct PathPoint {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

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

} // namespace frankx
