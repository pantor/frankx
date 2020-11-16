#include <frankx/robot.hpp>

#include <otgx/quintic.hpp>

#ifdef WITH_REFLEXXES
#include <otgx/reflexxes.hpp>
#endif


namespace frankx {

Robot::Robot(std::string fci_ip, double dynamic_rel): franka::Robot(fci_ip), fci_ip(fci_ip), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}

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

bool Robot::move(ImpedanceMotion& motion) {
    return move(Affine(), motion);
}

bool Robot::move(ImpedanceMotion& motion, MotionData& data) {
    return move(Affine(), motion, data);
}

bool Robot::move(const Affine& frame, ImpedanceMotion& motion) {
    auto data = MotionData();
    return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, ImpedanceMotion& motion, MotionData& data) {
    motion.move(this, frame, data);
}

bool Robot::move(JointMotion motion) {
    return move(Affine(), motion);
}

bool Robot::move(JointMotion motion, MotionData& data) {
    return move(Affine(), motion, data);
}

bool Robot::move(const Affine& frame, JointMotion motion) {
    auto data = MotionData();
    return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, JointMotion motion, MotionData& data) {
    motion.update(this, frame, data);
    try {
        control(motion);

    } catch (franka::Exception exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true;
}

bool Robot::move(WaypointMotion motion) {
    return move(Affine(), motion);
}

bool Robot::move(WaypointMotion motion, MotionData& data) {
    return move(Affine(), motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion motion) {
    auto data = MotionData();
    return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion motion, MotionData& data, bool repeat_on_error) {
    #ifdef WITH_REFLEXXES
    otgx::Reflexxes<degrees_of_freedoms> trajectory_generator(control_rate);
    #else
    otgx::Quintic<degrees_of_freedoms> trajectory_generator(control_rate);
    #endif

    otgx::InputParameter<degrees_of_freedoms> input_para;
    otgx::OutputParameter<degrees_of_freedoms> output_para;
    otgx::Result result;

    input_para.enabled = VectorCartRotElbow(true, true, true);
    setInputLimits(input_para, data);

    WaypointMotion current_motion = motion;
    auto waypoint_iterator = current_motion.waypoints.begin();
    bool waypoint_has_elbow = false;

    Vector7d old_vector = Vector7d::Zero();
    auto old_affine = Affine();
    double old_elbow = 0.0;

    double time = 0.0;
    auto motion_generator = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        if (time == 0.0) {
            franka::CartesianPose initial_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Vector7d initial_vector = Affine(initial_pose).vector_with_elbow(initial_pose.elbow[0]);
            Vector7d initial_velocity = (Vector7d() << robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5], robot_state.delbow_c[0]).finished();

            old_affine = Affine(initial_pose);
            old_vector = initial_vector;
            old_elbow = old_vector(6);

            input_para.current_position = initial_vector;
            input_para.current_velocity = initial_velocity;
            input_para.current_acceleration = Vector7d::Zero();

            std::cout << "initial position: " << input_para.current_position << std::endl;
            std::cout << "initial velocity: " << input_para.current_velocity << std::endl;
            std::cout << "initial acceleration: " << input_para.current_acceleration << std::endl;

            const Waypoint current_waypoint = *waypoint_iterator;
            waypoint_has_elbow = current_waypoint.elbow.has_value();
            auto target_position_vector = current_waypoint.getTargetVector(frame, old_affine, old_elbow);

            input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
            input_para.target_position = target_position_vector;
            input_para.target_velocity = Vector7d::Zero();
            setInputLimits(input_para, current_waypoint, data);

            std::cout << "target position: " << input_para.target_position << std::endl;
            std::cout << "target velocity: " << input_para.target_velocity << std::endl;

            old_affine = current_waypoint.getTargetAffine(frame, old_affine);
            old_vector = target_position_vector;
            old_elbow = old_vector(6);
        }

        /* for (auto& reaction : data.reactions) {
            if (reaction.has_fired) {
                continue;
            }

            if (reaction.condition.callback(robot_state, time)) {
                std::cout << "[frankx] reaction fired." << std::endl;
                reaction.has_fired = true;

                bool new_motion = false;

                if (reaction.waypoint_action.has_value()) {
                    new_motion = true;
                    current_motion = reaction.waypoint_action.value()(robot_state, time);
                } else if (reaction.waypoint_motion.has_value()) {
                    new_motion = true;
                    current_motion = *(reaction.waypoint_motion.value());
                }
#ifdef WITH_PYTHON
                else {
                    stop();
                }
#endif

                if (new_motion) {
                    waypoint_iterator = current_motion.waypoints.begin();

                    auto current_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
                    auto current_vector = Affine(current_pose).vector_with_elbow(current_pose.elbow[0]);
                    old_affine = Affine(current_pose);
                    old_vector = current_vector;
                    old_elbow = old_vector(6);

                    const Waypoint current_waypoint = *waypoint_iterator;
                    waypoint_has_elbow = current_waypoint.elbow.has_value();
                    auto target_position_vector = current_waypoint.getTargetVector(Affine(), old_affine, old_elbow);

                    input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
                    input_para.target_position = target_position_vector;
                    input_para.target_velocity = Vector7d::Zero();
                    setInputLimits(input_para, current_waypoint, data);

                    old_affine = current_waypoint.getTargetAffine(Affine(), old_affine);
                    old_vector = target_position_vector;
                    old_elbow = old_vector(6);
                } else {
                    return franka::MotionFinished(CartesianPose(input_para.current_position, waypoint_has_elbow));
                }
            }
        } */

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input_para, output_para);

            if (current_motion.reload || result == otgx::Result::Finished) {
                if (waypoint_iterator != current_motion.waypoints.end()) {
                    waypoint_iterator += 1;
                }

                if (current_motion.reload) {
                    waypoint_iterator = current_motion.waypoints.begin();
                    current_motion.reload = false;

                } else if (waypoint_iterator == current_motion.waypoints.end()) {
                    return franka::MotionFinished(CartesianPose(input_para.target_position, waypoint_has_elbow));
                }

                const Waypoint current_waypoint = *waypoint_iterator;
                waypoint_has_elbow = current_waypoint.elbow.has_value();
                auto target_position_vector = current_waypoint.getTargetVector(frame, old_affine, old_elbow);

                input_para.enabled = {true, true, true, true, true, true, waypoint_has_elbow};
                input_para.target_position = target_position_vector;
                input_para.target_velocity = Vector7d::Zero();
                setInputLimits(input_para, current_waypoint, data);

                old_affine = current_waypoint.getTargetAffine(frame, old_affine);
                old_vector = target_position_vector;
                old_elbow = old_vector(6);

            } else if (result == otgx::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(CartesianPose(input_para.current_position, waypoint_has_elbow));
            }

            // std::cout << "new position: " << output_para.new_position << std::endl;
            // std::cout << "new velocity: " << output_para.new_velocity << std::endl;

            input_para.current_position = output_para.new_position;
            input_para.current_velocity = output_para.new_velocity;
            input_para.current_acceleration = output_para.new_acceleration;
        }

        return CartesianPose(output_para.new_position, waypoint_has_elbow);
    };

    try {
        control(motion_generator, controller_mode);

    } catch (franka::Exception exception) {
        auto errors = readOnce().last_motion_errors;
        if (repeat_on_error && errors.cartesian_motion_generator_joint_acceleration_discontinuity || errors.cartesian_motion_generator_joint_velocity_discontinuity || errors.cartesian_motion_generator_velocity_discontinuity || errors.cartesian_motion_generator_acceleration_discontinuity) {
            std::cout << "[frankx robot] continue motion" << std::endl;

            auto robot_state = readOnce();
            franka::CartesianPose initial_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Vector7d initial_vector = Affine(initial_pose).vector_with_elbow(initial_pose.elbow[0]);
            Vector7d initial_velocity = (Vector7d() << robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5], robot_state.delbow_c[0]).finished();

            data.velocity_rel *= 0.4;
            data.acceleration_rel *= 0.4;
            data.jerk_rel *= 0.4;
            input_para.current_position = initial_vector;
            input_para.current_velocity = initial_velocity;
            input_para.current_acceleration = Vector7d::Zero();
            setInputLimits(input_para, *waypoint_iterator, data);

            automaticErrorRecovery();

            try {
                control(motion_generator, controller_mode);
            } catch (franka::Exception exception) {
                std::cout << exception.what() << std::endl;
                return false;
            }
            return true;
        }
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true;
}

void Robot::setInputLimits(otgx::InputParameter<7>& input_parameters, const MotionData& data) {
    setInputLimits(input_parameters, Waypoint(), data);
}

void Robot::setInputLimits(otgx::InputParameter<7>& input_parameters, const Waypoint& waypoint, const MotionData& data) {
    constexpr double translation_factor {0.5};
    constexpr double elbow_factor {0.32};
    constexpr double derivative_factor {0.4};

    if (waypoint.max_dynamics || data.max_dynamics) {
        setVector(input_parameters.max_velocity, VectorCartRotElbow(
            0.8 * translation_factor * max_translation_velocity,
            max_rotation_velocity,
            0.5 * elbow_factor * max_elbow_velocity
        ));
        setVector(input_parameters.max_acceleration, VectorCartRotElbow(
            translation_factor * derivative_factor * max_translation_acceleration,
            derivative_factor * max_rotation_acceleration,
            elbow_factor * derivative_factor * max_elbow_acceleration
        ));
        setVector(input_parameters.max_jerk, VectorCartRotElbow(
            translation_factor * derivative_factor * max_translation_jerk,
            derivative_factor * max_rotation_jerk,
            elbow_factor * derivative_factor * max_elbow_jerk
        ));

    } else {
        setVector(input_parameters.max_velocity, VectorCartRotElbow(
            translation_factor * waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_translation_velocity,
            waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_rotation_velocity,
            elbow_factor * waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_elbow_velocity
        ));
        setVector(input_parameters.max_acceleration, VectorCartRotElbow(
            translation_factor * derivative_factor * data.acceleration_rel * acceleration_rel * max_translation_acceleration,
            derivative_factor * data.acceleration_rel * acceleration_rel * max_rotation_acceleration,
            elbow_factor * derivative_factor * data.acceleration_rel * acceleration_rel * max_elbow_acceleration
        ));
        setVector(input_parameters.max_jerk, VectorCartRotElbow(
            translation_factor * std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_translation_jerk,
            std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_rotation_jerk,
            elbow_factor * std::pow(derivative_factor, 2) * data.jerk_rel * jerk_rel * max_elbow_jerk
        ));

        if (waypoint.minimum_time.has_value()) {
            input_parameters.minimum_duration = waypoint.minimum_time.value();
        }
    }
}

} // namepace frankx
