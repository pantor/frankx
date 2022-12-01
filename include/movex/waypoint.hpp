#pragma once

#include <optional>

#include <Eigen/Geometry>

#include <affx/affine.hpp>


namespace movex {

struct Waypoint {
    using Affine = affx::Affine;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    enum class ReferenceType {
        Absolute,
        Relative
    };

    Affine affine;
    std::optional<double> elbow;
    ReferenceType reference_type;


    //! Dynamic Waypoint: Relative velocity factor
    double velocity_rel {1.0};

    //! Dynamic Waypoint: Use maximal dynamics of the robot independent on other parameters
    bool max_dynamics {false};

    //! Zero velocity Waypoint: Stop to zero velocity
    bool zero_velocity {false};

    //! Dynamic Waypoint: Minimum time to get to next waypoint
    std::optional<double> minimum_time;

    //! Path Waypoint: Maximum distance for blending.
    double blend_max_distance {0.0};


    explicit Waypoint(): affine(Affine()), reference_type(ReferenceType::Absolute) {}
    explicit Waypoint(const Affine& affine, ReferenceType reference_type = ReferenceType::Absolute): affine(affine), reference_type(reference_type) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type = ReferenceType::Absolute): affine(affine), reference_type(reference_type), elbow(elbow) {}

    explicit Waypoint(bool zero_velocity): affine(Affine()), reference_type(ReferenceType::Relative), zero_velocity(zero_velocity) {}
    explicit Waypoint(double minimum_time): affine(Affine()), reference_type(ReferenceType::Relative), minimum_time(minimum_time) {}
    explicit Waypoint(const Affine& affine, ReferenceType reference_type, double velocity_rel): affine(affine), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type, double velocity_rel): affine(affine), elbow(elbow), reference_type(reference_type), velocity_rel(velocity_rel) {}

    // explicit Waypoint(const Affine& affine, double blend_max_distance): affine(affine), blend_max_distance(blend_max_distance) {}
    explicit Waypoint(const Affine& affine, std::optional<double> elbow, double blend_max_distance): affine(affine), elbow(elbow), blend_max_distance(blend_max_distance), reference_type(ReferenceType::Absolute) {}
    explicit Waypoint(const Affine& affine, double velocity_rel, double blend_max_distance, std::optional<double> elbow): affine(affine), velocity_rel(velocity_rel), blend_max_distance(blend_max_distance), elbow(elbow), reference_type(ReferenceType::Absolute) {}

    Affine getTargetAffine(const Affine& frame, const Affine& old_affine) const {
        switch (reference_type) {
            case ReferenceType::Absolute:
                return affine * frame.inverse();
            case ReferenceType::Relative:
                return old_affine * affine * frame.inverse();
        }
    }

    Vector7d getTargetVector(const Affine& old_affine, double old_elbow) const {
        return getTargetVector(Affine(), old_affine, old_elbow);
    }

    Vector7d getTargetVector(const Affine& frame, const Affine& old_affine, double old_elbow) const {
        double new_elbow;
        if (reference_type == ReferenceType::Relative) {
            new_elbow = elbow.value_or(0.0) + old_elbow;
        } else {
            new_elbow = elbow.value_or(old_elbow);
        }
        return getTargetAffine(frame, old_affine).vector_with_elbow(new_elbow);
    }
};

} // namespace movex
