#pragma once

#include <map>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <affx/affine.hpp>
#include <movex/robot/motion_data.hpp>


namespace movex {

struct ImpedanceMotion {
    using Affine = affx::Affine;

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
    const double joint_stiffness {200.0};  // ?

    Affine target;
    bool is_active {false};
    bool should_finish {false};

    explicit ImpedanceMotion() { }
    explicit ImpedanceMotion(double joint_stiffness): joint_stiffness(joint_stiffness), type(Type::Joint) { }
    explicit ImpedanceMotion(double translational_stiffness, double rotational_stiffness): translational_stiffness(translational_stiffness), rotational_stiffness(rotational_stiffness), type(Type::Cartesian) { }


    Affine getTarget() const {
        return target;
    }

    void setTarget(const Affine& new_target) {
        if (is_active) {
            target = new_target;
        }
        target_motion = ImpedanceMotion::TargetMotion::Exponential;
    }

    void setLinearRelativeTargetMotion(const Affine& relative_target, double duration) {
        linear_motion = {relative_target, duration};
        target_motion = ImpedanceMotion::TargetMotion::Linear;
    }

    void setSpiralTargetMotion(const Affine& center, double revolutions_per_second, double radius_per_revolution) {
        spiral_motion = {center, revolutions_per_second, radius_per_revolution};
        target_motion = ImpedanceMotion::TargetMotion::Spiral;
    }

    void addForceConstraint(Axis axis, double value) {
        if (is_active) {
            return;
        }

        force_constraints[axis] = value;
    }

    void addForceConstraint(std::optional<double> x = std::nullopt, std::optional<double> y = std::nullopt, std::optional<double> z = std::nullopt) {
        if (is_active) {
            return;
        }

        if (x) {
            force_constraints[Axis::X] = x.value();
        }
        if (y) {
            force_constraints[Axis::Y] = y.value();
        }
        if (z) {
            force_constraints[Axis::Z] = z.value();
        }
    }

    bool isActive() const {
        return is_active;
    }

    void finish() {
        should_finish = true;
    }
};

} // namespace movex
