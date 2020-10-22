#include <frankx/motion_joint.hpp>


namespace frankx {

JointMotion::JointMotion(const std::array<double, 7> q_goal): q_goal(q_goal) { }
JointMotion::JointMotion(const std::array<double, 7> q_goal, const std::array<double, 7> dq_goal): q_goal(q_goal), dq_goal(dq_goal) { }

void JointMotion::update(Robot* robot, const Affine& frame, const MotionData& motion_data) {
    this->robot = robot;
    this->motion_data = motion_data;

    dq_max = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
    ddq_max = (Vector7d() << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0).finished();
    dddq_max = (Vector7d() << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0).finished();

    dq_max *= motion_data.velocity_rel * robot->velocity_rel;
    ddq_max *= motion_data.acceleration_rel * robot->acceleration_rel;
    dddq_max *= motion_data.jerk_rel * robot->jerk_rel;
}

franka::JointPositions JointMotion::operator()(const franka::RobotState& robot_state, franka::Duration period) {
    time += period.toSec();
    if (time == 0.0) {
        setVector(input_parameters->CurrentPositionVector, robot_state.q);
        setVector(input_parameters->CurrentVelocityVector, robot_state.dq);
        setVector(input_parameters->CurrentAccelerationVector, robot_state.ddq_d);

        setVector(input_parameters->MaxVelocityVector, dq_max);
        setVector(input_parameters->MaxAccelerationVector, ddq_max);
        setVector(input_parameters->MaxJerkVector, dddq_max);

        setVector(input_parameters->TargetPositionVector, q_goal);
        setVector(input_parameters->TargetVelocityVector, dq_goal);
    }

#ifdef WITH_PYTHON
    if (PyErr_CheckSignals() == -1) {
        robot->stop();
    }
#endif

    const int steps = std::max<int>(period.toMSec(), 1);
    for (int i = 0; i < steps; i++) {
        result_value = rml->RMLPosition(*input_parameters, output_parameters.get(), flags);

        if (result_value == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
            return franka::MotionFinished(franka::JointPositions(q_goal));

        } else if (result_value == ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES) {
            std::cout << "Invalid inputs:" << std::endl;
            return franka::MotionFinished(franka::JointPositions(robot_state.q));
        }

        *input_parameters->CurrentPositionVector = *output_parameters->NewPositionVector;
        *input_parameters->CurrentVelocityVector = *output_parameters->NewVelocityVector;
        *input_parameters->CurrentAccelerationVector = *output_parameters->NewAccelerationVector;
    }

    std::array<double, 7> result;
    setVector(output_parameters->NewPositionVector, result);
    return franka::JointPositions(result);
}

} // namespace frankx
