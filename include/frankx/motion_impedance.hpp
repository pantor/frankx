#pragma once
// Inpspired by: https://github.com/frankaemika/libfranka/blob/develop/examples/cartesian_impedance_control.cpp

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Eigen/Core>
#ifdef WITH_PYTHON
    #include <Python.h>
#endif

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot_state.h>

#include <frankx/affine.hpp>
#include <frankx/motion_data.hpp>
#include <frankx/utils.hpp>
#include <frankx/robot.hpp>


namespace frankx {

class Robot;

class ImpedanceMotion {
    bool is_active {false};

    Affine frame;
    MotionData motion_data;
    Robot* robot;

public:
    const double translational_stiffness {150.0};
    const double rotational_stiffness {10.0};

    Affine target;

    explicit ImpedanceMotion() { }

    void setTarget(Affine new_target);
    bool isActive() const;

    franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);
    void update(Robot* robot, const Affine& frame, const MotionData& motion_data);
};

} // namespace frankx