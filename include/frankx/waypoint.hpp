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
    bool max_dynamics {false};

    std::optional<double> minimum_time;

    explicit Waypoint() {}
    explicit Waypoint(double minimum_time): minimum_time(minimum_time), reference_type(ReferenceType::Relative) {}
    explicit Waypoint(const Affine& affine, ReferenceType reference_type = ReferenceType::Absolute, double velocity_rel = 1.0): affine(affine), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type = ReferenceType::Absolute, double velocity_rel = 1.0): affine(affine), elbow(elbow), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, ReferenceType reference_type, bool max_dynamics): affine(affine), elbow(elbow), reference_type(reference_type), max_dynamics(max_dynamics) {}
    explicit Waypoint(const Affine& affine, const Vector7d& velocity, ReferenceType reference_type = ReferenceType::Absolute, double velocity_rel = 1.0): affine(affine), velocity(velocity), reference_type(reference_type), velocity_rel(velocity_rel) {}
    explicit Waypoint(const Affine& affine, double elbow, const Vector7d& velocity, ReferenceType reference_type = ReferenceType::Absolute, double velocity_rel = 1.0): affine(affine), elbow(elbow), velocity(velocity), reference_type(reference_type), velocity_rel(velocity_rel) {}

    Affine getTargetAffine(const Affine& frame, const Affine& old_affine) const {
        if (reference_type == ReferenceType::Relative) {
          return old_affine * affine * frame.inverse();
        }
        return affine * frame.inverse();
    }

    Vector7d getTargetVector(const Affine& frame, const Affine& old_affine, double old_elbow) const {
        double new_elbow;
        if (reference_type == ReferenceType::Relative) {
            new_elbow = elbow.has_value() ? elbow.value() + old_elbow : old_elbow;
        } else {
            new_elbow = elbow.has_value() ? elbow.value() : old_elbow;
        }
        return getTargetAffine(frame, old_affine).vector_with_elbow(new_elbow);
    }
};

} // namespace frankx