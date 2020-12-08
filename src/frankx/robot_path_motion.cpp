#include <frankx/robot.hpp>


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
    movex::Ruckig<1> trajectory_generator(control_rate);

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

} // namepace frankx
