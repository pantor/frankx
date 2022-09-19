#include <frankx/robot.hpp>

#include <ruckig/ruckig.hpp>


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

    // libfranka 0.8 split F_T_EE into F_T_NE (set in desk to value below) and NE_T_EE which defaults to identity
    // set it anyway again.
//    setEE({1, 0, 0, 0,
//           0, 1, 0, 0,
//           0, 0, 1, 0,
//           0, 0, 0, 1});
        setEE({1, 0, 0, 0,
           0, -1, 0, 0,
           0, 0, -1, 0,
           0, 0, 0, 1});
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

Affine Robot::currentPose(const Affine& frame, const bool& read_once) {
    ::franka::RobotState state;
    if (read_once)
        state = readOnce();
    else
        state = ::franka::RobotState(*asynchronous_state_ptr);
    return Affine(state.O_T_EE) * frame;
}

std::array<double, 7> Robot::currentJointPositions(const bool& read_once) {
    ::franka::RobotState state;
    if (read_once)
        state = readOnce();
    else
        state = ::franka::RobotState(*asynchronous_state_ptr);
    return state.q;
}

Affine Robot::forwardKinematics(const std::array<double, 7>& q) {
    const Eigen::Matrix<double, 7, 1> q_current = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q.data(), q.size());
    return Affine(Kinematics::forward(q_current));
}

std::array<double, 7> Robot::inverseKinematics(const Affine& target, const std::array<double, 7>& q0) {
    std::array<double, 7> result;

    Eigen::Matrix<double, 6, 1> x_target = target.vector();
    const Eigen::Matrix<double, 7, 1> q0_current = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q0.data(), q0.size());

    Eigen::Matrix<double, 7, 1>::Map(result.data()) = Kinematics::inverse(x_target, q0_current);
    return result;
}

::franka::RobotState Robot::get_state(const bool& read_once) {
    if (read_once)
        return readOnce();
    else
        return ::franka::RobotState(*asynchronous_state_ptr);
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
    ImpedanceMotionGenerator<Robot> mg {this, frame, motion, data};

    asynchronous_state_ptr = &mg.asynchronous_state;

    try {
        control(stateful<franka::Torques>(mg));
        motion.is_active = false;

    } catch (const franka::Exception& exception) {
        std::cout << exception.what() << std::endl;
        motion.is_active = false;
        return false;
    }
    return true;
}


bool Robot::move(JointMotion motion) {
    auto data = MotionData();
    return move(motion, data);
}

bool Robot::move(JointMotion motion, MotionData& data) {
    JointMotionGenerator<Robot> mg {this, motion, data};

    asynchronous_state_ptr = &mg.asynchronous_state;

    try {
        control(stateful<franka::JointPositions>(mg));

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
    PathMotionGenerator<Robot> mg {this, frame, motion, data};

    asynchronous_state_ptr = &mg.asynchronous_state;

    try {
        control(stateful<franka::CartesianPose>(mg), controller_mode);

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
    WaypointMotionGenerator<Robot> mg {this, frame, motion, data};
    mg.input_para.target_position[0] = 0.01;

    asynchronous_state_ptr = &mg.asynchronous_state;

    try {
        control(stateful<franka::CartesianPose>(mg), controller_mode);

    } catch (franka::Exception exception) {
        auto errors = readOnce().last_motion_errors;
        if (repeat_on_error
            // && (errors.cartesian_motion_generator_joint_acceleration_discontinuity
            // || errors.cartesian_motion_generator_joint_velocity_discontinuity
            // || errors.cartesian_motion_generator_velocity_discontinuity
            // || errors.cartesian_motion_generator_acceleration_discontinuity)
        ) {
            std::cout << "[frankx robot] continue motion after exception: " << exception.what() << std::endl;
            automaticErrorRecovery();

            data.velocity_rel *= 0.5;
            data.acceleration_rel *= 0.5;
            data.jerk_rel *= 0.5;
            mg.reset();

            bool success {false};

            try {
                control(stateful<franka::CartesianPose>(mg), controller_mode);
                success = true;

            } catch (franka::Exception exception) {
                std::cout << exception.what() << std::endl;
            }
            data.velocity_rel *= 2;
            data.acceleration_rel *= 2;
            data.jerk_rel *= 2;

            return success;

        } else {
            std::cout << exception.what() << std::endl;
        }

        return false;
    }
    return true;
}

} // namepace frankx
