#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_joint.hpp>
#include <movex/otg/smoothie.hpp>


namespace frankx {
    using namespace movex;

template<class RobotType>
struct JointMotionGenerator: public MotionGenerator {
    movex::Smoothie<RobotType::degrees_of_freedoms> trajectory_generator {RobotType::control_rate};

    movex::InputParameter<RobotType::degrees_of_freedoms> input_para;
    movex::OutputParameter<RobotType::degrees_of_freedoms> output_para;
    movex::Result result;

    std::array<double, RobotType::degrees_of_freedoms> joint_positions;
    double time {0.0};
    RobotType* robot;

    JointMotion motion;
    MotionData& data;

    explicit JointMotionGenerator(RobotType* robot, JointMotion motion, MotionData& data): robot(robot), motion(motion), data(data) { }

    void init(const franka::RobotState& robot_state, franka::Duration period) {
        input_para.current_position = Vector7d(robot_state.q_d.data());
        input_para.current_velocity = Vector7d::Zero();
        input_para.current_acceleration = Vector7d::Zero();

        input_para.target_position = motion.target;
        input_para.target_velocity = Vector7d::Zero();
        input_para.target_acceleration = Vector7d::Zero();

        input_para.max_velocity = Vector7d(RobotType::max_joint_velocity.data());
        input_para.max_acceleration = 0.3 * Vector7d(RobotType::max_joint_acceleration.data());

        input_para.max_velocity *= robot->velocity_rel * data.velocity_rel;
        input_para.max_acceleration *= robot->acceleration_rel * data.acceleration_rel;
    }

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period) {
        time += period.toSec();
        if (time == 0.0) {
            init(robot_state, period);
        }

#ifdef WITH_PYTHON
        if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            robot->stop();
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
    }
};

} // namespace frankx
