#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_path.hpp>
#include <movex/path/path.hpp>
#include <movex/path/time_parametrization.hpp>
#include <movex/path/trajectory.hpp>


namespace frankx {
    using namespace movex;

template<class RobotType>
struct PathMotionGenerator: public MotionGenerator {
    size_t trajectory_index {0};
    double s_current {0.0};
    const bool use_elbow {false};
    double time {0.0};

    Trajectory trajectory {};

    RobotType* robot;
    Affine frame;
    PathMotion motion;
    MotionData& data;
    franka::RobotState asynchronous_state;

    explicit PathMotionGenerator(RobotType* robot, const Affine& frame, PathMotion motion, MotionData& data): robot(robot), frame(frame), motion(motion), data(data) {
        // Insert current pose into beginning of path
        auto initial_state = robot->readOnce();
        franka::CartesianPose initial_cartesian_pose(initial_state.O_T_EE_c, initial_state.elbow_c);
        Affine initial_pose(initial_cartesian_pose.O_T_EE);

        Waypoint start_waypoint {initial_pose * frame, initial_cartesian_pose.elbow[0]};
        auto all_waypoints = motion.waypoints;
        all_waypoints.insert(all_waypoints.begin(), start_waypoint);

        // Create path
        const Path path {all_waypoints};

        // Get time parametrization
        TimeParametrization time_parametrization {RobotType::control_rate};
        const auto [max_velocity, max_acceleration, max_jerk] = getInputLimits(robot, data);
        trajectory = time_parametrization.parametrize(path, max_velocity, max_acceleration, max_jerk);
    }

    franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period) {
        time += period.toSec();

#ifdef WITH_PYTHON
        if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            robot->stop();
        }
#endif

        asynchronous_state = franka::RobotState(robot_state);

        const int steps = std::max<int>(period.toMSec(), 1);
        trajectory_index += steps;
        if (trajectory_index >= trajectory.states.size()) {
            s_current = trajectory.path.get_length();
            return franka::MotionFinished(CartesianPose(trajectory.path.q(s_current, frame), use_elbow));
        }

        s_current = trajectory.states[trajectory_index].s;
        return CartesianPose(trajectory.path.q(s_current, frame), use_elbow);
    }
};

} // namespace frankx
