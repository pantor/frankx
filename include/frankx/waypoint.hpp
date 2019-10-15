#pragma once

#include <optional>

#include <frankx/utils.hpp>


namespace frankx {

class Waypoint {
  Affine affine {Affine()};
  double elbow {0.0};

  std::array<double, 7> velocity {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

public:
  enum class ReferenceType {
    Absolute,
    Relative
  } reference_type {ReferenceType::Absolute};

  std::optional<double> minimum_time;


  Waypoint() {}
  Waypoint(double minimum_time): minimum_time(minimum_time), reference_type(ReferenceType::Relative) {}
  Waypoint(const Affine& affine, double elbow): affine(affine), elbow(elbow) {}
  Waypoint(const Affine& affine, double elbow, ReferenceType reference_type): affine(affine), elbow(elbow), reference_type(reference_type) {}
  Waypoint(const Affine& affine, double elbow, const std::array<double, 7>& velocity): affine(affine), elbow(elbow), velocity(velocity) {}
  Waypoint(const Affine& affine, double elbow, const std::array<double, 7>& velocity, ReferenceType reference_type): affine(affine), elbow(elbow), velocity(velocity), reference_type(reference_type) {}

  Affine getTargetAffine(const Affine& old_affine) const {
    if (reference_type == ReferenceType::Relative) {
      return old_affine * affine;
    }
    return affine;
  }

  Vector7d getTargetVector(const Affine& old_affine, double old_elbow, const Vector7d& old_vector) const {
    double new_elbow = (reference_type == ReferenceType::Relative) ? old_elbow + elbow : elbow;
    return getTargetAffine(old_affine).vector(new_elbow, old_vector);
  }

  std::array<double, 7> getTargetVelocity() const {
    return velocity;
  }
};

} // namespace frankx