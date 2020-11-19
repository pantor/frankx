#pragma once

#include <memory>

#include <Eigen/Core>


namespace movex {

using Vector7d = Eigen::Matrix<double, 7, 1>;

struct Segment {
    double length;

    virtual double get_length() const = 0;
    virtual Vector7d q(double s) const = 0;
    virtual Vector7d pdq(double s) const = 0;
    virtual Vector7d pddq(double s) const = 0;
    virtual Vector7d pdddq(double s) const = 0;
};


class LineSegment: public Segment {
public:
    Vector7d start, end;

    explicit LineSegment(const Vector7d& start, const Vector7d&end): start(start), end(end) {
        Vector7d diff = end - start;

        length = diff.norm();
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
    double radius;

    explicit CircleSegment(const Vector7d& center, const Vector7d& start, double angle = 2 * M_PI) {

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


class QuinticSegment: public Segment {
    double s_length;

public:
    Vector7d a, b, c, d, e, f;

    explicit QuinticSegment(const Vector7d& a, const Vector7d& b, const Vector7d& c, const Vector7d& d, const Vector7d& e, const Vector7d& f, double s_length): a(a), b(b), c(c), d(d), e(e), f(f), s_length(s_length) {
        // Numerical integration here
        length = 1.0;
    }

    double get_length() const {
        return length;
    }

    Vector7d q(double s) const {
        return f + s * (e + s * (d + s * (c + s * (b + s * a))));
    }

    Vector7d pdq(double s) const {
        return e + s * (2 * d + s * (3 * c + s * (4 * b + s * 5 * a)));
    }

    Vector7d pddq(double s) const {
        return 2 * d + s * (6 * c + s * (12 * b + s * 20 * a));
    }

    Vector7d pdddq(double s) const {
        return 6 * c + s * (24 * b + s * 60 * a);
    }
};

} // namespace movex
