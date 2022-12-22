#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_joint.hpp>
#include <ruckig/ruckig.hpp>


namespace frankx {
    using namespace movex;

template<class RobotType>
struct JointMotionGenerator: public MotionGenerator {
    ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator {RobotType::control_rate};

    ruckig::InputParameter<RobotType::degrees_of_freedoms> input_para;
    ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_para;
    ruckig::Result result;

    std::array<double, RobotType::degrees_of_freedoms> joint_positions;
    double time {0.0};
    RobotType* robot;

    JointMotion motion;
    MotionData& data;
    franka::RobotState asynchronous_state;

    explicit JointMotionGenerator(RobotType* robot, JointMotion motion, MotionData& data): robot(robot), motion(motion), data(data) { }

    void init(const franka::RobotState& robot_state, franka::Duration period) {
        input_para.current_position = robot_state.q_d;
        input_para.current_velocity = toStd(Vector7d::Zero());
        input_para.current_acceleration = toStd(Vector7d::Zero());

        input_para.target_position = toStd(motion.target);
        input_para.target_velocity = toStd(Vector7d::Zero());
        input_para.target_acceleration = toStd(Vector7d::Zero());

        for (size_t dof = 0; dof < RobotType::degrees_of_freedoms; dof += 1) {
            input_para.max_velocity[dof] = RobotType::max_joint_velocity[dof] * robot->velocity_rel * data.velocity_rel;
            input_para.max_acceleration[dof] = 0.3 * RobotType::max_joint_acceleration[dof] * robot->acceleration_rel * data.acceleration_rel;
            input_para.max_jerk[dof] = 0.3 * RobotType::max_joint_jerk[dof] * robot->jerk_rel * data.jerk_rel;
        }
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

        asynchronous_state = franka::RobotState(robot_state);

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input_para, output_para);
            joint_positions = output_para.new_position;

            if (result == ruckig::Result::Finished) {
                joint_positions = input_para.target_position;
                return franka::MotionFinished(franka::JointPositions(joint_positions));

            } else if (result == ruckig::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(franka::JointPositions(joint_positions));
            }

            output_para.pass_to_input(input_para);
        }

        return franka::JointPositions(joint_positions);
    }
};

} // namespace frankx
