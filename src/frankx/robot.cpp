#include <frankx/robot.hpp>

#include <movex/otg/quintic.hpp>
#include <movex/otg/smoothie.hpp>


#ifdef WITH_REFLEXXES
#include <movex/otg/reflexxes.hpp>
#endif


namespace frankx {

Robot::Robot(std::string fci_ip, double dynamic_rel, bool repeat_on_error, bool stop_at_python_signal): franka::Robot(fci_ip), fci_ip(fci_ip), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel), repeat_on_error(repeat_on_error), stop_at_python_signal(stop_at_python_signal) { }

void Robot::setDefaultBehavior() {
    setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},
        {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0}}
    );

    // setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    // setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    setEE({0.7071, 0.7071, 0.0, 0.0, 0.7071, -0.7071, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

void Robot::setDynamicRel(double dynamic_rel) {
    velocity_rel = dynamic_rel;
    acceleration_rel = dynamic_rel;
    jerk_rel = dynamic_rel;
}

bool Robot::hasErrors() {
    return bool(readOnce().current_errors);
}

bool Robot::recoverFromErrors() {
    automaticErrorRecovery();
    return !hasErrors();
}

Affine Robot::currentPose(const Affine& frame) {
    auto state = readOnce();
    return Affine(state.O_T_EE) * frame;
}

// Affine Robot::forwardKinematics(const std::array<double, 7>& q) {
//     Affine result;

//     control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
//         franka::CartesianPose cartesian_target(robot_state.O_T_EE_c, robot_state.elbow_c);
//         result = Affine(cartesian_target.O_T_EE);
//         return franka::MotionFinished(franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

//     }, [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {

//         return franka::JointPositions(q);
//     });

//     return result;
// }

// std::array<double, 7> Robot::inverseKinematics(const Affine& target, const Affine& frame) {
//     std::array<double, 7> result;
//     return result;
// }

void Robot::setInputLimits(movex::InputParameter<7>& input_parameters, const MotionData& data) {
    setInputLimits(input_parameters, movex::Waypoint(), data);
}

void Robot::setInputLimits(movex::InputParameter<7>& input_parameters, const movex::Waypoint& waypoint, const MotionData& data) {
    constexpr double translation_factor {0.5};
    constexpr double elbow_factor {0.32};
    constexpr double derivative_factor {0.4};

    if (waypoint.max_dynamics || data.max_dynamics) {
        setCartRotElbowVector(input_parameters.max_velocity,
            0.8 * translation_factor * max_translation_velocity,
            max_rotation_velocity,
            0.5 * elbow_factor * max_elbow_velocity
        );
        setCartRotElbowVector(input_parameters.max_acceleration,
            translation_factor * derivative_factor * max_translation_acceleration,
            derivative_factor * max_rotation_acceleration,
            elbow_factor * derivative_factor * max_elbow_acceleration
        );
        setCartRotElbowVector(input_parameters.max_jerk,
            translation_factor * derivative_factor * max_translation_jerk,
            derivative_factor * max_rotation_jerk,
            elbow_factor * derivative_factor * max_elbow_jerk
        );

    } else {
        setCartRotElbowVector(input_parameters.max_velocity,
            translation_factor * waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_translation_velocity,
            waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_rotation_velocity,
            elbow_factor * waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_elbow_velocity
        );
        setCartRotElbowVector(input_parameters.max_acceleration,
            translation_factor * derivative_factor * data.acceleration_rel * acceleration_rel * max_translation_acceleration,
            derivative_factor * data.acceleration_rel * acceleration_rel * max_rotation_acceleration,
            elbow_factor * derivative_factor * data.acceleration_rel * acceleration_rel * max_elbow_acceleration
        );
        setCartRotElbowVector(input_parameters.max_jerk,
            translation_factor * std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_translation_jerk,
            std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_rotation_jerk,
            elbow_factor * std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_elbow_jerk
        );

        if (waypoint.minimum_time.has_value()) {
            input_parameters.minimum_duration = waypoint.minimum_time.value();
        }
    }
}

} // namepace frankx
