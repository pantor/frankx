#include <frankx/robot.hpp>


namespace frankx {

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

} // namepace frankx
