#pragma once

#include <array>
#include <iostream>

#include <Eigen/Core>
#ifdef WITH_PYTHON
    #include <Python.h>
#endif

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include <frankx/motion_data.hpp>
#include <frankx/utils.hpp>
#include <frankx/robot.hpp>
#include <otgx/reflexxes.hpp>
#include <otgx/quintic.hpp>


namespace frankx {

class Robot;

class JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    double time {0.0};

    Vector7d dq_max, ddq_max, dddq_max;

    RMLPositionFlags flags;
    int result_value = 0;

    const std::shared_ptr<ReflexxesAPI> rml;
    std::shared_ptr<RMLPositionInputParameters> input_parameters;
    std::shared_ptr<RMLPositionOutputParameters> output_parameters;

    MotionData motion_data;
    Robot* robot;

public:
    std::array<double, 7> q_goal;
    std::array<double, 7> dq_goal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    explicit JointMotion(const std::array<double, 7> q_goal);
    explicit JointMotion(const std::array<double, 7> q_goal, const std::array<double, 7> dq_goal);

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);
    void update(Robot* robot, const Affine& frame, const MotionData& motion_data);
};

} // namespace frankx
