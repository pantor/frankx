#pragma once

#include <array>
#include <iostream>

#include <Eigen/Core>
#ifdef WITH_PYTHON
    #include <Python.h>
#endif

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include <frankx/motion_data.hpp>
#include <frankx/utils.hpp>


namespace frankx {

class JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    double time {0.0};

    Vector7d dq_max, ddq_max, dddq_max;

    RMLPositionFlags flags;
    int result_value = 0;

    const std::shared_ptr<ReflexxesAPI> rml;
    std::shared_ptr<RMLPositionInputParameters> input_parameters;
    std::shared_ptr<RMLPositionOutputParameters> output_parameters;

public:
    std::array<double, 7> q_goal;
    std::array<double, 7> dq_goal {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    explicit JointMotion(const std::array<double, 7> q_goal);
    explicit JointMotion(const std::array<double, 7> q_goal, const std::array<double, 7> dq_goal);

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

    void setDynamicRel(double velocity_rel, double acceleration_rel, double jerk_rel);
};

} // namespace frankx
