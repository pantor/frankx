#pragma once

#include <chrono>
#include <iostream>
#include <optional>

#include <movex/otg/parameter.hpp>


namespace movex {

struct Profile {
    enum class Type {
        UP_ACC0_ACC1_VEL, UP_VEL, UP_ACC0, UP_ACC1, UP_ACC0_ACC1, UP_ACC0_VEL, UP_ACC1_VEL, UP_NONE,
        DOWN_ACC0_ACC1_VEL, DOWN_VEL, DOWN_ACC0, DOWN_ACC1, DOWN_ACC0_ACC1, DOWN_ACC0_VEL, DOWN_ACC1_VEL, DOWN_NONE
    };

    Type type;
    std::array<double, 7> t, t_sum, j;
    std::array<double, 8> a, v, p;

    //! Total time of the braking segments
    std::optional<double> t_brake;

    //! Allow up to two segments of braking before the "correct" profile starts
    std::array<double, 2> t_brakes, j_brakes, a_brakes, v_brakes, p_brakes;

    void set(double p0, double v0, double a0, std::array<double, 7> j);
    void reset(double p0, double v0, double a0, double base_jerk);
    bool check(double pf, double vf, double vMax, double aMax) const;

    static std::tuple<double, double, double> integrate(double t, double p0, double v0, double a0, double j);
};


struct RuckigEquation {
    static bool time_up_acc0_acc1_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_acc0(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_acc1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_acc0_acc1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_acc0_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_acc1_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_up_none(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);

    static bool time_down_acc0_acc1_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_acc0(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_acc1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_acc0_acc1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_acc0_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_acc1_vel(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);
    static bool time_down_none(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);

    static bool get_profile(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax);

    static void get_brake_trajectory(double v0, double a0, double vMax, double aMax, double jMax, std::array<double, 2>& t_brake, std::array<double, 2>& j_brake);

    static double jerk_to_reach_target_with_times(const std::array<double, 7>& t, double p0, double v0, double a0, double pf);
};


template<size_t DOFs>
class Ruckig {
    InputParameter<DOFs> current_input;

    double t, tf;
    std::array<Profile, DOFs> profiles;

    bool calculate(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        current_input = input;

        // Check input
        if ((input.max_velocity.array() <= 0.0).any() || (input.max_acceleration.array() <= 0.0).any() || (input.max_jerk.array() <= 0.0).any()) {
            return false;
        }

        if (DOFs > 1 && (input.target_velocity.array() != 0.0).any()) {
            std::cerr << "Ruckig does not support a target velocity for multiple DoFs." << std::endl;
            return false;
        }

        if ((input.target_velocity.array().abs() > input.max_velocity.array()).any()) {
            std::cerr << "Target velocity exceeds maximal velocity." << std::endl;
            return false;
        }

        if ((input.target_acceleration.array() != 0.0).any()) {
            std::cerr << "Ruckig does not support a target acceleration." << std::endl;
            return false;
        }

        if (input.minimum_duration.has_value()) {
            std::cerr << "Ruckig does not support a minimum duration." << std::endl;
            return false;
        }

        auto start = std::chrono::high_resolution_clock::now();

        // Calculate brakes (if input exceeds or will exceed limits)
        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof]) {
                continue;
            }

            RuckigEquation::get_brake_trajectory(input.current_velocity[dof], input.current_acceleration[dof], input.max_velocity[dof], input.max_acceleration[dof], input.max_jerk[dof], profiles[dof].t_brakes, profiles[dof].j_brakes);
            profiles[dof].t_brake = profiles[dof].t_brakes[0] + profiles[dof].t_brakes[1];

            // std::cout << dof << ": " << t_brakes_[dof][0] << " " << t_brakes_[dof][1] << std::endl;
        }

        std::array<double, DOFs> tfs;
        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof]) {
                tfs[dof] = 0.0;
                continue;
            }

            double p0 = input.current_position[dof];
            double v0 = input.current_velocity[dof];
            double a0 = input.current_acceleration[dof];

            if (profiles[dof].t_brakes[0] > 0.0) {
                profiles[dof].p_brakes[0] = p0;
                profiles[dof].v_brakes[0] = v0;
                profiles[dof].a_brakes[0] = a0;
                std::tie(p0, v0, a0) = Profile::integrate(profiles[dof].t_brakes[0], p0, v0, a0, profiles[dof].j_brakes[0]);

                if (profiles[dof].t_brakes[1] > 0.0) {
                    profiles[dof].p_brakes[1] = p0;
                    profiles[dof].v_brakes[1] = v0;
                    profiles[dof].a_brakes[1] = a0;
                    std::tie(p0, v0, a0) = Profile::integrate(profiles[dof].t_brakes[1], p0, v0, a0, profiles[dof].j_brakes[1]);
                }
            }

            if (!RuckigEquation::get_profile(profiles[dof], p0, v0, a0, input.target_position[dof], input.target_velocity[dof], input.max_velocity[dof], input.max_acceleration[dof], input.max_jerk[dof])) {
                throw std::runtime_error("Error while calculating an online trajectory for: "
                    + std::to_string(input.current_position[dof]) + ", " + std::to_string(input.current_velocity[dof]) + ", " + std::to_string(input.current_acceleration[dof])
                    + " targets: " + std::to_string(input.target_position[dof])
                    + " limits: " + std::to_string(input.max_velocity[dof]) + ", " + std::to_string(input.max_acceleration[dof]) + ", " + std::to_string(input.max_jerk[dof])
                    + " profile input: " + std::to_string(p0) + ", " + std::to_string(v0) + ", " + std::to_string(a0)
                );
            }
            tfs[dof] = profiles[dof].t_sum[6] + profiles[dof].t_brake.value_or(0.0);
        }

        auto tf_max_pointer = std::max_element(tfs.begin(), tfs.end());
        size_t limiting_dof = std::distance(tfs.begin(), tf_max_pointer);
        tf = *tf_max_pointer;

        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof] || dof == limiting_dof) {
                continue;
            }

            // ToDo: Synchronize multiple DoFs
            // double new_jerk = RuckigEquation::jerk_to_reach_target_with_times(profiles[limiting_dof].t, input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], input.target_position[dof]);
            // profiles[dof].t = profiles[limiting_dof].t;
            // profiles[dof].reset(input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], new_jerk);
        }

        auto stop = std::chrono::high_resolution_clock::now();
        last_calculation_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1000.0;

        t = 0.0;
        output.duration = tf;
        return true;
    }

public:
    //! Time step between updates (cycle time) in [s]
    const double delta_time;

    //! Time for calculating the last full trajectory in [µs]
    double last_calculation_duration {-1};

    explicit Ruckig(double delta_time): delta_time(delta_time) { }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        t += delta_time;

        if (input != current_input && !calculate(input, output)) {
            return Result::Error;
        }

        if (t + delta_time > tf) {
            output.new_position = input.target_position;
            output.new_velocity = input.target_velocity;
            output.new_acceleration = input.target_acceleration;
            return Result::Finished;
        }

        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof]) {
                output.new_acceleration[dof] = input.current_acceleration[dof];
                output.new_velocity[dof] = input.current_velocity[dof];
                output.new_position[dof] = input.current_position[dof];
            }

            auto& p = profiles[dof];

            double t_diff = t;
            if (p.t_brake.has_value()) {
                if (t_diff < p.t_brake.value()) {
                    size_t index = (t_diff < p.t_brakes[0]) ? 0 : 1;
                    if (index > 0) {
                        t_diff -= p.t_brakes[index - 1];
                    }

                    std::tie(output.new_position[dof], output.new_velocity[dof], output.new_acceleration[dof]) = Profile::integrate(t_diff, p.p_brakes[index], p.v_brakes[index], p.a_brakes[index], p.j_brakes[index]);
                    continue;
                } else {
                    t_diff -= p.t_brake.value();
                }
            }

            if (t_diff >= p.t_sum[6]) {
                output.new_position[dof] = p.p[7];
                output.new_velocity[dof] = p.v[7];
                output.new_acceleration[dof] = p.a[7];
                continue;
            }

            auto index_ptr = std::upper_bound(p.t_sum.begin(), p.t_sum.end(), t_diff);
            size_t index = std::distance(p.t_sum.begin(), index_ptr);

            if (index > 0) {
                t_diff -= p.t_sum[index - 1];
            }

            std::tie(output.new_position[dof], output.new_velocity[dof], output.new_acceleration[dof]) = Profile::integrate(t_diff, p.p[index], p.v[index], p.a[index], p.j[index]);
        }

        current_input.current_position = output.new_position;
        current_input.current_velocity = output.new_velocity;
        current_input.current_acceleration = output.new_acceleration;
        return Result::Working;
    }
};

} // namespace movex
