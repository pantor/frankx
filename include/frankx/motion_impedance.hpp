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
    Affine frame;
    MotionData motion_data;
    Robot* robot;

public:
    const double translational_stiffness {2000.0};  // in [10, 3000] N/m
    const double rotational_stiffness {200.0};  // in [1, 300] Nm/rad

    Affine target;
    bool is_active {false};
    bool should_finish {false};

    double filter_params {0.005};

    explicit ImpedanceMotion() { }
    explicit ImpedanceMotion(double translational_stiffness, double rotational_stiffness): translational_stiffness(translational_stiffness), rotational_stiffness(rotational_stiffness) { }

    Affine getTarget() const;
    void setTarget(const Affine& new_target);
    bool isActive() const;
    void finish();
};

} // namespace frankx