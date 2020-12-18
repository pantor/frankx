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

    Vector7d dq(double s, double ds) const {
        return pdq(s) * ds;
    }

    Vector7d ddq(double s, double ds, double dds) const {
        return pddq(s) * std::pow(ds, 2) + pdq(s) * dds;
    }

    Vector7d dddq(double s, double ds, double dds, double ddds) const {
        return 3 * ds * pddq(s) * dds + std::pow(ds, 3) * pdddq(s) + pdq(s) * ddds;
    }

    virtual Vector7d max_pddq() const = 0;
    virtual Vector7d max_pdddq() const = 0;
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

    Vector7d max_pddq() const {
        return Vector7d::Zero();
    }

    Vector7d max_pdddq() const {
        return Vector7d::Zero();
    }
};


class CircleSegment: public Segment {

};


class QuinticSegment: public Segment {

};


class QuarticBlendSegment: public Segment {
    void integrate_path_length() {
        length = 0.0;

        Vector7d f_s = q(0.0), f_s_new;
        const size_t steps {5000};
        const double step_length = s_length / steps;
        const double step_length_squared = std::pow(step_length, 2);
        for (int i = 1; i < steps; i += 1) {
            f_s_new = q(i * step_length);
            length += std::sqrt((f_s_new - f_s).squaredNorm() + step_length_squared);
            std::swap(f_s_new, f_s);
        }
    }

public:
    double s_length;
    Vector7d b, c, e, f;
    Vector7d lb, lm, rb, rm;

    explicit QuarticBlendSegment(const Vector7d& lb, const Vector7d& lm, const Vector7d& rb, const Vector7d& rm, double s_mid, double max_diff, double s_abs_max): lb(lb), lm(lm), rb(rb), rm(rm) {
        Vector7d sAbs_ = ((-16*max_diff)/(3.*(lm - rm).array())).abs();
        double s_abs_min = std::min<double>({sAbs_.minCoeff(), s_abs_max});
        s_length = 2 * s_abs_min;

        b = (lm - rm).array() / (16.*std::pow(s_abs_min, 3));
        c = (-lm + rm).array() / (4.*std::pow(s_abs_min, 2));
        e = lm;
        f = lb.array() + lm.array()*(-s_abs_min + s_mid);
    }

    double get_length() const {
        return s_length;
    }

    Vector7d q(double s) const {
        return f + s * (e + s * (s * (c + s * b)));
    }

    Vector7d pdq(double s) const {
        return e + s * (s * (3 * c + s * 4 * b));
    }

    Vector7d pddq(double s) const {
        return s * (6 * c + s * 12 * b);
    }

    Vector7d pdddq(double s) const {
        return 6 * c + s * 24 * b;
    }

    Vector7d max_pddq() const {
        double s_abs = s_length / 2;
        return (-3*(lm - rm))/(4.*s_abs);
    }

    Vector7d max_pdddq() const {
        double s_abs = s_length / 2;
        return (3*(lm - rm))/(2.*std::pow(s_abs,2));
    }
};

} // namespace movex
