#pragma once

#include <Eigen/Core>
#include <franka/duration.h>
#include <franka/robot_state.h>

#include <ruckig/input_parameter.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/waypoint.hpp>


namespace frankx {
    using namespace movex;
    using Affine = affx::Affine;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;

struct MotionGenerator {
    static inline franka::CartesianPose CartesianPose(const Vector7d& vector, bool include_elbow = true) {
        auto affine = Affine(vector);
        if (include_elbow) {
            return franka::CartesianPose(affine.array(), {vector[6], -1});
        }
        return franka::CartesianPose(affine.array());
    }

    static inline franka::CartesianPose CartesianPose(const std::array<double, 7>& vector, bool include_elbow = true) {
        auto affine = Affine(vector);
        if (include_elbow) {
            return franka::CartesianPose(affine.array(), {vector[6], -1});
        }
        return franka::CartesianPose(affine.array());
    }

    template <class T = double>
    static inline std::array<T, 7> VectorCartRotElbow(T cart, T rot, T elbow) {
        return {cart, cart, cart, rot, rot, rot, elbow};
    }

    static inline void setCartRotElbowVector(Vector7d& vector, double cart, double rot, double elbow) {
        const std::array<double, 7> data = VectorCartRotElbow(cart, rot, elbow);
        vector = Eigen::Map<const Vector7d>(data.data(), data.size());
    }

    static inline std::array<double, 7> toStd(const Vector7d& vector) {
        std::array<double, 7> result;
        Vector7d::Map(result.data()) = vector;
        return result;
    }

    static inline movex::RobotState<7> convertState(const franka::RobotState& franka) {
        movex::RobotState<7> movex;
        movex.q = franka.q;
        movex.q_d = franka.q_d;
        movex.dq = franka.dq;

        movex.O_T_EE = franka.O_T_EE;
        movex.O_T_EE_c = franka.O_T_EE_c;
        movex.O_dP_EE_c = franka.O_dP_EE_c;

        movex.elbow = franka.elbow;
        movex.elbow_c = franka.elbow_c;
        movex.elbow_d = franka.elbow_d;

        movex.O_F_ext_hat_K = franka.O_F_ext_hat_K;
        return movex;
    }

    template<class RobotType>
    static std::tuple<std::array<double, 7>, std::array<double, 7>, std::array<double, 7>> getInputLimits(RobotType* robot, const MotionData& data) {
        return getInputLimits(robot, Waypoint(), data);
    }

    template<class RobotType>
    static std::tuple<std::array<double, 7>, std::array<double, 7>, std::array<double, 7>> getInputLimits(RobotType* robot, const Waypoint& waypoint, const MotionData& data) {
        constexpr double translation_factor {0.4};
        constexpr double derivative_factor {0.4};

        if (waypoint.max_dynamics || data.max_dynamics) {
            auto max_velocity = MotionGenerator::VectorCartRotElbow(
                0.8 * translation_factor * robot->max_translation_velocity,
                0.8 * robot->max_rotation_velocity,
                0.8 * robot->max_elbow_velocity
            );
            auto max_acceleration = MotionGenerator::VectorCartRotElbow(
                0.8 * translation_factor * derivative_factor * robot->max_translation_acceleration,
                0.8 * derivative_factor * robot->max_rotation_acceleration,
                0.8 * derivative_factor * robot->max_elbow_acceleration
            );
            auto max_jerk = MotionGenerator::VectorCartRotElbow(
                0.8 * translation_factor * std::pow(derivative_factor, 2) * robot->max_translation_jerk,
                0.8 * std::pow(derivative_factor, 2) * robot->max_rotation_jerk,
                0.8 * std::pow(derivative_factor, 2) * robot->max_elbow_jerk
            );
            return {max_velocity, max_acceleration, max_jerk};
        }

        auto max_velocity = MotionGenerator::VectorCartRotElbow(
            translation_factor * waypoint.velocity_rel * data.velocity_rel * robot->velocity_rel * robot->max_translation_velocity,
            waypoint.velocity_rel * data.velocity_rel * robot->velocity_rel * robot->max_rotation_velocity,
            waypoint.velocity_rel * data.velocity_rel * robot->velocity_rel * robot->max_elbow_velocity
        );
        auto max_acceleration = MotionGenerator::VectorCartRotElbow(
            translation_factor * derivative_factor * data.acceleration_rel * robot->acceleration_rel * robot->max_translation_acceleration,
            derivative_factor * data.acceleration_rel * robot->acceleration_rel * robot->max_rotation_acceleration,
            derivative_factor * data.acceleration_rel * robot->acceleration_rel * robot->max_elbow_acceleration
        );
        auto max_jerk = MotionGenerator::VectorCartRotElbow(
            translation_factor * std::pow(derivative_factor, 2) * data.jerk_rel * robot->jerk_rel * robot->max_translation_jerk,
            std::pow(derivative_factor, 2) * data.jerk_rel * robot->jerk_rel * robot->max_rotation_jerk,
            std::pow(derivative_factor, 2) * data.jerk_rel * robot->jerk_rel * robot->max_elbow_jerk
        );
        return {max_velocity, max_acceleration, max_jerk};
    }

    template<class RobotType>
    static void setInputLimits(ruckig::InputParameter<7>& input_parameters, RobotType* robot, const MotionData& data) {
        setInputLimits(input_parameters, robot, Waypoint(), data);
    }

    template<class RobotType>
    static void setInputLimits(ruckig::InputParameter<7>& input_parameters, RobotType* robot, const Waypoint& waypoint, const MotionData& data) {
        std::tie(input_parameters.max_velocity, input_parameters.max_acceleration, input_parameters.max_jerk) = getInputLimits(robot, waypoint, data);

        if (!(waypoint.max_dynamics || data.max_dynamics) && waypoint.minimum_time.has_value()) {
            input_parameters.minimum_duration = waypoint.minimum_time.value();
        }

        if (waypoint.max_dynamics) {
            input_parameters.synchronization = ruckig::Synchronization::TimeIfNecessary;
        } else {
            input_parameters.synchronization = ruckig::Synchronization::Time;
        }
    }
};


// Stateful functor to get values after the control function
template <typename OriginalFunctor, typename RType>
class StatefulFunctor {
    OriginalFunctor &fun;

public:
    StatefulFunctor() = delete;
    StatefulFunctor(OriginalFunctor &orig) : fun(orig) {}
    StatefulFunctor(StatefulFunctor const &other) : fun(other.fun) {}
    StatefulFunctor(StatefulFunctor &&other) : fun(other.fun) {}

    template <typename... Args>
    RType operator() (Args&&... args) {
        return fun(std::forward<Args>(args)...);
    }
};


template <typename RT, typename OF>
StatefulFunctor<OF, RT> stateful(OF &fun) {
    return StatefulFunctor<OF, RT>(fun);
}

} // namespace frankx
