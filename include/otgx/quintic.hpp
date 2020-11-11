#pragma once

#include <Eigen/Core>

#include <otgx/parameter.hpp>


namespace otgx {

template<size_t DOFs>
class Quintic {
    using Vector7d = Eigen::Matrix<double, DOFs, 1>;

    // Trajectory
    Vector7d a, b, c, d, e, f;
    double t, tf;
    InputParameter<DOFs> current_input;

    bool calculate(const InputParameter<DOFs>& input) {
        current_input = input;

        const Vector7d& x0 = input.current_position;
        const Vector7d& v0 = input.current_velocity;
        const Vector7d& a0 = input.current_acceleration;
        const Vector7d& xf = input.target_position;
        const Vector7d& vf = input.target_velocity;
        const Vector7d& af = input.target_acceleration;
        const Vector7d& v_max = input.max_velocity;
        const Vector7d& a_max = input.max_acceleration;
        const Vector7d& j_max = input.max_jerk;

        // Approximations for v0 == 0, vf == 0, a0 == 0, af == 0
        Vector7d v_max_tfs = (15 * (x0 - xf).array().abs()) / (8 * v_max).array();
        Vector7d a_max_tfs = (std::sqrt(10) * (x0.array().pow(2) - 2 * x0.array() * xf.array() + xf.array().pow(2)).pow(1./4)) / (std::pow(3, 1./4) * a_max.array().sqrt());
        Vector7d j_max_tfs = ((60 * (x0 - xf).array().abs()) / j_max.array()).pow(1./3);

        tf = std::max<double>({v_max_tfs.maxCoeff(), a_max_tfs.maxCoeff(), j_max_tfs.maxCoeff()});
        t = 0.0;

        a = -((a0 - af) * std::pow(tf, 2) + 6 * tf * v0 + 6 * tf * vf + 12 * x0 - 12 * xf) / (2 * std::pow(tf, 5));
        b = -((2 * af - 3 * a0) * std::pow(tf, 2) - 16 * tf * v0 - 14 * tf * vf - 30 * x0 + 30 * xf) / (2 * std::pow(tf, 4));
        c = -((3 * a0 - af) * std::pow(tf, 2) + 12 * tf * v0 + 8 * tf * vf + 20 * x0 - 20 * xf) / (2 * std::pow(tf, 3));
        d = a0 / 2;
        e = v0;
        f = x0;

        return true;
    }

public:
    double delta_time;

    explicit Quintic(double delta_time): delta_time(delta_time) { }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        t += delta_time;

        if (input != current_input && !calculate(input)) {
            return Result::Error;
        }

        if (t >= tf) {
            output.new_position = input.target_position;
            output.new_velocity = input.target_velocity;
            output.new_acceleration = input.target_acceleration;
            return Result::Finished;
        }

        output.new_position = f + t * (e + t * (d + t * (c + t * (b + a * t))));
        output.new_velocity = e + t * (2 * d + t * (3 * c + t * (4 * b + 5 * a * t)));
        output.new_acceleration = 2 * d + t * (6 * c + t * (12 * b + t * (20 * a)));

        current_input.current_position = output.new_position;
        current_input.current_velocity = output.new_velocity;
        current_input.current_acceleration = output.new_acceleration;
        return Result::Working;
    }
};

} // namespace otgx
