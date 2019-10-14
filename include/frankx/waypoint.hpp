#pragma once

#ifndef FRANKX_WAYPOINT_HPP
#define FRANKX_WAYPOINT_HPP

#include <frankx/geometry.hpp>


class Waypoint {
  Eigen::Affine3d affine {Eigen::Affine3d::Identity()};
  double elbow {0.0};

  std::array<double, 7> velocity {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

public:
  enum class ReferenceType {
    Absolute,
    Relative
  } reference_type {ReferenceType::Absolute};

  Waypoint() {}
  Waypoint(const Eigen::Affine3d& affine, double elbow): affine(affine), elbow(elbow) {}
  Waypoint(const Eigen::Affine3d& affine, double elbow, ReferenceType reference_type): affine(affine), elbow(elbow), reference_type(reference_type) {}
  Waypoint(const Eigen::Affine3d& affine, double elbow, const std::array<double, 7>& velocity): affine(affine), elbow(elbow), velocity(velocity) {}
  Waypoint(const Eigen::Affine3d& affine, double elbow, const std::array<double, 7>& velocity, ReferenceType reference_type): affine(affine), elbow(elbow), velocity(velocity), reference_type(reference_type) {}

  Eigen::Affine3d getTargetAffine(const Eigen::Affine3d& old_affine) {
    if (reference_type == ReferenceType::Relative) {
      return old_affine * affine;
    }
    return affine;
  }

  Vector7d getTargetVector(const Eigen::Affine3d& old_affine, double old_elbow, const Vector7d& old_vector) {
    double new_elbow = (reference_type == ReferenceType::Relative) ? old_elbow + elbow : elbow;
    return Vector(getTargetAffine(old_affine), new_elbow, old_vector);
  }

  std::array<double, 7> getTargetVelocity() {
    return velocity;
  }
};

#endif // FRANKX_WAYPOINT_HPP
