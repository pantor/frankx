#pragma once

#include <optional>

#include <frankx/affine.hpp>
#include <frankx/utils.hpp>


namespace frankx {

struct Waypoint {
    enum class ReferenceType {
        Absolute,
        Relative
    };

    Affine affine {Affine()};
    Vector7d velocity {Vector7d::Zero()};
    std::optional<double> elbow;
    ReferenceType reference_type {ReferenceType::Absolute};

    double velocity_rel {1.0};
    double acceleration_rel {1.0};
    double jerk_rel {1.0};

    std::optional<double> minimum_time;

    explicit Waypoint() {}
    explicit Waypoint(double minimum_time): minimum_time(minimum_time), reference_type(ReferenceType::Relative) {}
    Waypoint(const Affine& affine, ReferenceType reference_type = ReferenceType::Absolute, double dynamic_rel = 1.0): affine(affine), velocity(velocity), reference_type(reference_type), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}
    Waypoint(const Affine& affine, double elbow, ReferenceType reference_type = ReferenceType::Absolute, double dynamic_rel = 1.0): affine(affine), elbow(elbow), velocity(velocity), reference_type(reference_type), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}
    explicit Waypoint(const Affine& affine, const Vector7d& velocity, ReferenceType reference_type = ReferenceType::Absolute, double dynamic_rel = 1.0): affine(affine), velocity(velocity), reference_type(reference_type), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, const Vector7d& velocity, ReferenceType reference_type = ReferenceType::Absolute, double dynamic_rel = 1.0): affine(affine), elbow(elbow), reference_type(reference_type), velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) {}

    Affine getTargetAffine(const Affine& old_affine) const {
        if (reference_type == ReferenceType::Relative) {
          return old_affine * affine;
        }
        return affine;
    }

    Vector7d getTargetVector(const Affine& old_affine, double old_elbow, const Vector7d& old_vector) const {
        double new_elbow = elbow.has_value() ? elbow.value() : 0.0;
        if (reference_type == ReferenceType::Relative) {
            new_elbow += old_elbow;
        }
        return getTargetAffine(old_affine).vector_with_elbow(new_elbow, old_vector);
    }
};

} // namespace frankx