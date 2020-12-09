#include <frankx/robot.hpp>
#include <movex/path/time_parametrization.hpp>


namespace frankx {

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
    // Insert current pose into beginning of path
    auto initial_state = readOnce();
    franka::CartesianPose initial_cartesian_pose(initial_state.O_T_EE_c, initial_state.elbow_c);
    Affine initial_pose(initial_cartesian_pose.O_T_EE);

    Waypoint start_waypoint {initial_pose * frame, initial_cartesian_pose.elbow[0]};
    auto all_waypoints = motion.waypoints;
    all_waypoints.insert(all_waypoints.begin(), start_waypoint);

    // Create path
    const Path path {all_waypoints};

    // Get time parametrization
    TimeParametrization time_parametrization {control_rate};
    const auto [max_velocity, max_acceleration, max_jerk] = getInputLimits(data);
    const auto trajectory = time_parametrization.parametrize(path, max_velocity, max_acceleration, max_jerk);

    const bool use_elbow {false};

    double time {0.0};
    size_t trajectory_index {0};
    double s_current {0.0};
    auto motion_generator = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();

#ifdef WITH_PYTHON
        if (stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            stop();
        }
#endif

        const int steps = std::max<int>(period.toMSec(), 1);
        trajectory_index += steps;
        if (trajectory_index >= trajectory.states.size()) {
            s_current = path.get_length();
            return franka::MotionFinished(CartesianPose(path.q(s_current, frame), use_elbow));
        }

        s_current = trajectory.states[trajectory_index].s;
        return CartesianPose(path.q(s_current, frame), use_elbow);
    };

    try {
        control(motion_generator, controller_mode);

    } catch (franka::Exception exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true;
}

} // namepace frankx
