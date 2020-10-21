#pragma once

#include <Eigen/Core>


namespace frankx {

template<size_t DOFs>
class TrajectoryGenerator {
    enum Result {
        Working,
        Finished,
        Error
    };

    typedef Eigen::Matrix<double, DOFs, 1> Vector;

    // Current trajectory
    Vector x0, v0, a0;
    Vector xf, vf, af;

    Vector a, b, c, d, e, f;
    double t, tf;

public:
    double delta_time;

    TrajectoryGenerator(double delta_time): delta_time(delta_time) { }

    bool calculate(Vector x0, Vector v0, Vector a0, Vector xf, Vector vf, Vector af, Vector v_max) {
        this->x0 = x0;
        this->v0 = v0;
        this->a0 = a0;
        this->xf = xf;
        this->vf = vf;
        this->af = af;

        auto tfs = (15 * std::abs(x0 - xf)) / (8 * v_max);  // Approximation for v0 == 0, vf == 0, a0 == 0, af == 0
        tf = tfs.maxCoeff();
        t = 0.0;

        a = -((a0 - af) * std::pow(tf, 2) + 6 * tf * v0 + 6 * tf * vf + 12 * x0 - 12 * xf) / (2 * std::pow(tf, 5));
        b = -((2 * af - 3 * a0) * std::pow(tf, 2) - 16 * tf * v0 - 14 * tf * vf - 30 * x0 + 30 * xf) / (2 * std::pow(tf, 4));
        c = -((3 * a0 - af) * std::pow(tf, 2) + 12 * tf * v0 + 8 * tf * vf + 20 * x0 - 20 * xf) / (2 * std::pow(tf, 3));
        d = a0 / 2;
        e = v0;
        f = x0;

        return true;
    }

    Result update(Vector& pos_output, Vector& vel_output, Vector& acc_output) {
        t += delta_time;

        if (t >= tf) {
            pos_output = xf;
            vel_output = vf;
            acc_output = af;
            return Result::Finished;
        }

        pos_output = f + t * (e + t * (d + t * (c + t * (b + a * t))));
        vel_output = e + t * (2 * d + t * (3 * c + t * (4 * b + 5 * a * t)));
        acc_output = 2 * d + t * (6 * c + t * (12 * b + t * (20 * a)));
        return Result::Working;
    }
};

} // namespace frankx
