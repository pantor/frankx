#include <frankx/robot.hpp>


namespace frankx {

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

            const auto current_waypoint = *waypoint_iterator;
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

            if (reaction.condition(convertState(robot_state), time)) {
                std::cout << "[frankx] reaction fired." << std::endl;
                reaction.has_fired = true;

                bool new_motion = false;

                if (reaction.waypoint_action.has_value()) {
                    new_motion = true;
                    current_motion = reaction.waypoint_action.value()(convertState(robot_state), time);
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

                    const auto current_waypoint = *waypoint_iterator;
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
                bool has_new_waypoint {false};

                if (waypoint_iterator != current_motion.waypoints.end()) {
                    waypoint_iterator += 1;
                    has_new_waypoint = (waypoint_iterator != current_motion.waypoints.end());
                }

                if (motion.return_when_finished && waypoint_iterator == current_motion.waypoints.end()) {
                    return franka::MotionFinished(CartesianPose(input_para.target_position, waypoint_has_elbow));

                } else if (motion.reload) {
                    current_motion = motion;
                    waypoint_iterator = current_motion.waypoints.begin();
                    motion.reload = false;
                    current_motion.reload = false;
                    has_new_waypoint = true;
                }

                if (has_new_waypoint) {
                    const auto current_waypoint = *waypoint_iterator;
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

} // namepace frankx
