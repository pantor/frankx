#pragma once
// Inpspired by: https://github.com/frankaemika/libfranka/blob/develop/examples/cartesian_impedance_control.cpp

#include <map>
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

#include <movex/affine.hpp>
#include <frankx/measure.hpp>
#include <frankx/motion_data.hpp>
#include <frankx/utils.hpp>
#include <frankx/robot.hpp>


namespace frankx {

class Robot;

struct ImpedanceMotion {
    enum class Axis { X, Y, Z };

    enum class Type {
        Cartesian,
        Joint
    };

    enum class TargetMotion {
        Exponential,
        Linear,
        Spiral,
    };

    struct LinearTargetMotion {
        Affine relative_target;
        double duration;
        bool initialized {false};
        bool finish_after {true};
        double finish_wait_factor {1.2};
    };

    struct SpiralTargetMotion {
        Affine center; // In XY-plane
        double revolutions_per_second;
        double radius_per_revolution;
        bool initialized {false};
    };

    Type type;

    TargetMotion target_motion {TargetMotion::Exponential};
    double exponential_decay {0.005};
    LinearTargetMotion linear_motion;
    SpiralTargetMotion spiral_motion;

    std::map<Axis, double> force_constraints;

public:
    const double translational_stiffness {2000.0};  // in [10, 3000] N/m
    const double rotational_stiffness {200.0};  // in [1, 300] Nm/rad
    double joint_stiffness {200.0};  // ?

    Affine target;
    bool is_active {false};
    bool should_finish {false};

    explicit ImpedanceMotion() { }
    explicit ImpedanceMotion(double joint_stiffness): joint_stiffness(joint_stiffness), type(Type::Joint) { }
    explicit ImpedanceMotion(double translational_stiffness, double rotational_stiffness): translational_stiffness(translational_stiffness), rotational_stiffness(rotational_stiffness), type(Type::Cartesian) { }

    Affine getTarget() const;
    void setTarget(const Affine& new_target);
    void setLinearRelativeTargetMotion(const Affine& relative_target, double duration);
    void setSpiralTargetMotion(const Affine& center, double revolutions_per_second, double radius_per_revolution);

    void addForceConstraint(Axis axis, double value);
    void addForceConstraint(std::optional<double> x = std::nullopt, std::optional<double> y = std::nullopt, std::optional<double> z = std::nullopt);
    bool isActive() const;
    void finish();

    bool move(Robot* robot, const Affine& frame, MotionData& data);
};

} // namespace frankx