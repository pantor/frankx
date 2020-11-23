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
    movex::Smoothie<degrees_of_freedoms> trajectory_generator(control_rate);

    movex::InputParameter<degrees_of_freedoms> input_para;
    movex::OutputParameter<degrees_of_freedoms> output_para;
    movex::Result result;

    std::array<double, 7> joint_positions;

    double time {0.0};
    auto motion_generator = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
        time += period.toSec();
        if (time == 0.0) {
            input_para.current_position = Vector7d(robot_state.q_d.data());
            input_para.current_velocity = Vector7d::Zero();
            input_para.current_acceleration = Vector7d::Zero();

            input_para.target_position = motion.target;
            input_para.target_velocity = Vector7d::Zero();
            input_para.target_acceleration = Vector7d::Zero();

            input_para.max_velocity = Vector7d(max_joint_velocity.data());
            input_para.max_acceleration = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();

            input_para.max_velocity *= velocity_rel * data.velocity_rel;
            input_para.max_acceleration *= acceleration_rel * data.acceleration_rel;
        }

#ifdef WITH_PYTHON
        if (stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            stop();
        }
#endif

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input_para, output_para);
            Eigen::VectorXd::Map(&joint_positions[0], 7) = output_para.new_position;

            if (result == movex::Result::Finished) {
                Eigen::VectorXd::Map(&joint_positions[0], 7) = input_para.target_position;
                return franka::MotionFinished(franka::JointPositions(joint_positions));

            } else if (result == movex::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(franka::JointPositions(joint_positions));
            }

            input_para.current_position = output_para.new_position;
            input_para.current_velocity = output_para.new_velocity;
            input_para.current_acceleration = output_para.new_acceleration;
        }

        return franka::JointPositions(joint_positions);
    };

    try {
        control(motion_generator);

    } catch (franka::Exception exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true;
}

bool Robot::move(PathMotion motion) {
    return move(Affine(), motion);
}

bool Robot::move(PathMotion motion, MotionData& data) {
    return move(Affine(), motion, data);
}

bool Robot::move(const Affine& frame, PathMotion motion) {
    auto data = MotionData();
    return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, PathMotion motion, MotionData& data) {
    movex::Quintic<1> trajectory_generator(control_rate);

    movex::InputParameter<1> input;
    movex::OutputParameter<1> output;
    movex::Result result;

    std::unique_ptr<movex::Path> path;

    bool use_elbow {false};

    double time = 0.0;
    auto motion_generator = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        if (time == 0.0) {
            franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Affine initial_pose(initial_cartesian_pose.O_T_EE);

            auto all_waypoints = motion.waypoints;
            all_waypoints.insert(all_waypoints.begin(), initial_pose * frame.inverse());

            path = std::make_unique<movex::Path>(all_waypoints, motion.blend_max_distance);

            input.current_position(0) = 0.0;
            input.current_velocity(0) = 0.0;
            input.current_acceleration(0) = 0.0;
            input.target_position(0) = path->get_length();
            input.target_velocity(0) = 0.0;
            input.max_velocity(0) = velocity_rel;
            input.max_acceleration(0) = 2 * acceleration_rel;
            input.max_jerk(0) = 5 * jerk_rel;
        }

#ifdef WITH_PYTHON
        if (stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            stop();
        }
#endif

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input, output);

            if (result == movex::Result::Finished) {
                return franka::MotionFinished(CartesianPose(path->q(input.target_position(0)), use_elbow));

            } else if (result == movex::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(CartesianPose(path->q(input.current_position(0)), use_elbow));
            }

            input.current_position = output.new_position;
            input.current_velocity = output.new_velocity;
            input.current_acceleration = output.new_acceleration;
        }

        return CartesianPose(path->q(output.new_position(0)), use_elbow);
    };

    try {
        control(motion_generator, controller_mode);

    } catch (franka::Exception exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true;
}

bool Robot::move(WaypointMotion& motion) {
    return move(Affine(), motion);
}

bool Robot::move(WaypointMotion& motion, MotionData& data) {
    return move(Affine(), motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion& motion) {
    auto data = MotionData();
    return move(frame, motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion& motion, MotionData& data) {
#ifdef WITH_REFLEXXES
    movex::Reflexxes<degrees_of_freedoms> trajectory_generator(control_rate);
#else
    movex::Quintic<degrees_of_freedoms> trajectory_generator(control_rate);
#endif

    movex::InputParameter<degrees_of_freedoms> input_para;
    movex::OutputParameter<degrees_of_freedoms> output_para;
    movex::Result result;

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
            franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Affine initial_pose(initial_cartesian_pose.O_T_EE);

            Vector7d initial_vector = initial_pose.vector_with_elbow(initial_cartesian_pose.elbow[0]);
            Vector7d initial_velocity = (Vector7d() << robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5], robot_state.delbow_c[0]).finished();

            old_affine = initial_pose;
            old_vector = initial_vector;
            old_elbow = old_vector(6);

            input_para.current_position = initial_vector;
            input_para.current_velocity = initial_velocity;
            input_para.current_acceleration = Vector7d::Zero();

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
        }

        for (auto& reaction : data.reactions) {
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

                    franka::CartesianPose current_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
                    Affine current_pose(current_cartesian_pose.O_T_EE);
                    auto current_vector = current_pose.vector_with_elbow(current_cartesian_pose.elbow[0]);
                    old_affine = current_pose;
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
        }

#ifdef WITH_PYTHON
        if (stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            stop();
        }
#endif

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input_para, output_para);

            if (motion.reload || result == movex::Result::Finished) {
                if (waypoint_iterator != current_motion.waypoints.end()) {
                    waypoint_iterator += 1;
                }

                if (motion.return_when_finished && waypoint_iterator == current_motion.waypoints.end()) {
                    return franka::MotionFinished(CartesianPose(input_para.target_position, waypoint_has_elbow));

                } else if (motion.reload) {
                    current_motion = motion;
                    waypoint_iterator = current_motion.waypoints.begin();
                    motion.reload = false;
                    current_motion.reload = false;

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

            } else if (result == movex::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(CartesianPose(input_para.current_position, waypoint_has_elbow));
            }

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
            franka::CartesianPose initial_cartesian_pose(robot_state.O_T_EE_c, robot_state.elbow_c);
            Affine initial_pose(initial_cartesian_pose.O_T_EE);

            Vector7d initial_vector = initial_pose.vector_with_elbow(initial_cartesian_pose.elbow[0]);
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

void Robot::setInputLimits(movex::InputParameter<7>& input_parameters, const MotionData& data) {
    setInputLimits(input_parameters, Waypoint(), data);
}

void Robot::setInputLimits(movex::InputParameter<7>& input_parameters, const Waypoint& waypoint, const MotionData& data) {
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
