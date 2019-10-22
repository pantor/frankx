#include <frankx/robot.hpp>


namespace frankx {

Robot::Robot(std::string fci_ip, double dynamic_rel): franka::Robot(fci_ip), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}

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
    setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
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
    auto affine = Affine(state.O_T_EE_c);
    affine.data.rotate(affine.offset_euler);
    return affine * frame;
}

bool Robot::move(JointMotion motion) {
    auto data = MotionData();
    return move(motion, data);
}

bool Robot::move(JointMotion motion, MotionData& data) {
    motion.setDynamicRel(data.velocity_rel * velocity_rel);
    control(motion);
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
    return move(motion, data);
}

bool Robot::move(const Affine& frame, WaypointMotion motion, MotionData& data) {
    constexpr int degrees_of_freedoms {7};
    constexpr double control_rate {0.001};

    RMLPositionFlags flags;
    int result_value = 0;

    const auto rml = std::make_shared<ReflexxesAPI>(degrees_of_freedoms, control_rate);
    auto input_parameters = std::make_shared<RMLPositionInputParameters>(degrees_of_freedoms);
    auto output_parameters = std::make_shared<RMLPositionOutputParameters>(degrees_of_freedoms);

    setVector(input_parameters->SelectionVector, VectorCartRotElbow(true, true, true));
    setVector(input_parameters->MaxVelocityVector, VectorCartRotElbow(
        data.velocity_rel * velocity_rel * max_translation_velocity,
        data.velocity_rel * velocity_rel * max_rotation_velocity,
        data.velocity_rel * velocity_rel * max_elbow_velocity
    ));
    setVector(input_parameters->MaxAccelerationVector, VectorCartRotElbow(
        data.acceleration_rel * acceleration_rel * max_translation_acceleration,
        data.acceleration_rel * acceleration_rel * max_rotation_acceleration,
        data.acceleration_rel * acceleration_rel * max_elbow_acceleration
    ));
    setVector(input_parameters->MaxJerkVector, VectorCartRotElbow(
        data.jerk_rel * jerk_rel * max_translation_jerk,
        data.jerk_rel * jerk_rel * max_rotation_jerk,
        data.jerk_rel * jerk_rel * max_elbow_jerk
    ));

    WaypointMotion current_motion = motion;
    auto waypoint_iterator = current_motion.waypoints.begin();
    bool waypoint_has_elbow = false;

    Vector7d old_vector = Vector7d::Zero();
    auto old_affine = Affine();
    double old_elbow = 0.0;

    double time = 0.0;
    try {
        control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
            time += period.toSec();
            if (time == 0.0) {
                franka::CartesianPose initial_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
                Vector7d initial_velocity = (Vector7d() << robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5], robot_state.delbow_c[0]).finished();

                Vector7d initial_vector = Affine(initial_pose).vector_with_elbow(initial_pose.elbow[0], old_vector);
                old_affine = Affine(initial_pose);
                old_vector = initial_vector;
                old_elbow = old_vector(6);

                setVector(input_parameters->CurrentPositionVector, initial_vector);
                setVector(input_parameters->CurrentVelocityVector, initial_velocity);
                setZero(input_parameters->CurrentAccelerationVector);

                const Waypoint current_waypoint = *waypoint_iterator;
                waypoint_has_elbow = current_waypoint.elbow.has_value();
                auto target_position_vector = current_waypoint.getTargetVector(frame, old_affine, old_elbow, old_vector);

                setInputLimits(input_parameters.get(), current_waypoint, data);
                setVector(input_parameters->TargetPositionVector, target_position_vector);
                setVector(input_parameters->TargetVelocityVector, current_waypoint.velocity);

                old_affine = current_waypoint.getTargetAffine(frame, old_affine);
                old_vector = target_position_vector;
                old_elbow = old_vector(6);
            }

            for (auto& reaction : data.reactions) {
                if (reaction.has_fired) {
                    continue;
                }

                if (reaction.condition_callback(robot_state, time)) {
                    reaction.has_fired = true;

                    bool new_motion = false;

                    if (reaction.action.has_value()) {
                        new_motion = true;
                        current_motion = reaction.action.value()(robot_state, time);
                    } else if (reaction.motion.has_value()) {
                        new_motion = true;
                        current_motion = *(reaction.motion.value());
                    }

                    if (new_motion) {
                        waypoint_iterator = current_motion.waypoints.begin();

                        auto current_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
                        auto current_vector = Affine(current_pose).vector_with_elbow(current_pose.elbow[0], old_vector);
                        old_affine = Affine(current_pose);
                        old_vector = current_vector;
                        old_elbow = old_vector(6);

                        const Waypoint current_waypoint = *waypoint_iterator;
                        waypoint_has_elbow = current_waypoint.elbow.has_value();
                        auto target_position_vector = current_waypoint.getTargetVector(frame, old_affine, old_elbow, old_vector);

                        setInputLimits(input_parameters.get(), current_waypoint, data);
                        setVector(input_parameters->TargetPositionVector, target_position_vector);
                        setVector(input_parameters->TargetVelocityVector, current_waypoint.velocity);

                        old_affine = current_waypoint.getTargetAffine(frame, old_affine);
                        old_vector = target_position_vector;
                        old_elbow = old_vector(6);
                    } else {
                        return franka::MotionFinished(CartesianPose(input_parameters->CurrentPositionVector, waypoint_has_elbow));
                    }
                }
            }

#ifdef WITH_PYTHON
            if (PyErr_CheckSignals() == -1) {
                stop();
            }
#endif

            const int steps = std::max<int>(period.toMSec(), 1);
            for (int i = 0; i < steps; i++) {
                result_value = rml->RMLPosition(*input_parameters, output_parameters.get(), flags);

                if (current_motion.reload || result_value == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
                    waypoint_iterator += 1;

                    if (current_motion.reload) {
                        waypoint_iterator = current_motion.waypoints.begin();
                        current_motion.reload = false;

                    } else if (waypoint_iterator == current_motion.waypoints.end()) {
                        return franka::MotionFinished(CartesianPose(input_parameters->CurrentPositionVector, waypoint_has_elbow));
                    }

                    const Waypoint current_waypoint = *waypoint_iterator;
                    waypoint_has_elbow = current_waypoint.elbow.has_value();
                    auto target_position_vector = current_waypoint.getTargetVector(frame, old_affine, old_elbow, old_vector);

                    setInputLimits(input_parameters.get(), current_waypoint, data);
                    setVector(input_parameters->TargetPositionVector, target_position_vector);
                    setVector(input_parameters->TargetVelocityVector, current_waypoint.velocity);

                    old_affine = current_waypoint.getTargetAffine(frame, old_affine);
                    old_vector = target_position_vector;
                    old_elbow = old_vector(6);
                }

                *input_parameters->CurrentPositionVector = *output_parameters->NewPositionVector;
                *input_parameters->CurrentVelocityVector = *output_parameters->NewVelocityVector;
                *input_parameters->CurrentAccelerationVector = *output_parameters->NewAccelerationVector;
            }

            return CartesianPose(output_parameters->NewPositionVector, waypoint_has_elbow);
        }, franka::ControllerMode::kCartesianImpedance);

    } catch (franka::Exception exception) {
        // automaticErrorRecovery();
        std::cout << exception.what() << std::endl;
        return false;
    }

    return true;
}

void Robot::setInputLimits(RMLPositionInputParameters *input_parameters, const Waypoint& waypoint, const MotionData& data) {
    setVector(input_parameters->MaxVelocityVector, VectorCartRotElbow(
        waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_translation_velocity,
        waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_rotation_velocity,
        waypoint.velocity_rel * data.velocity_rel * velocity_rel * max_elbow_velocity
    ));
    setVector(input_parameters->MaxAccelerationVector, VectorCartRotElbow(
        waypoint.acceleration_rel * data.acceleration_rel * acceleration_rel * max_translation_acceleration,
        waypoint.acceleration_rel * data.acceleration_rel * acceleration_rel * max_rotation_acceleration,
        waypoint.acceleration_rel * data.acceleration_rel * acceleration_rel * max_elbow_acceleration
    ));
    setVector(input_parameters->MaxJerkVector, VectorCartRotElbow(
        waypoint.jerk_rel * data.jerk_rel * jerk_rel * max_translation_jerk,
        waypoint.jerk_rel * data.jerk_rel * jerk_rel * max_rotation_jerk,
        waypoint.jerk_rel * data.jerk_rel * jerk_rel * max_elbow_jerk
    ));

    if (waypoint.minimum_time.has_value()) {
        input_parameters->SetMinimumSynchronizationTime(waypoint.minimum_time.value());
    }
}

} // namepace frankx
