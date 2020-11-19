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
    Vector7d start, end;

public:
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


class QuinticPolynomialSegment: public Segment {
    Vector7d a, b, c, d, e, f;

public:
    explicit QuinticPolynomialSegment(const Vector7d& a, const Vector7d& b, const Vector7d& c, const Vector7d& d, const Vector7d& e, const Vector7d& f): a(a), b(b), c(c), d(d), e(e), f(f) {
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
        return 5 * a * std::pow(s, 4) + 4 * b * std::pow(s, 3) + 3 * c * std::pow(s, 2) + 2 * d * s + e;
    }

    Vector7d pddq(double s) const {
        return 20 * a * std::pow(s, 3) + 12 * b * std::pow(s, 2) + 6 * c * s + 2 * d;
    }

    Vector7d pdddq(double s) const {
        return 60 * a * std::pow(s, 2) + 24 * b * s + 6 * c;
    }
};


class SplineSegment: public Segment {

};

} // namespace movex
