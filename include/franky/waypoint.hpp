#pragma once

#include <optional>

#include <Eigen/Geometry>

#include <franky/robot_pose.hpp>

#include <franky/types.hpp>


namespace franky {

  struct Waypoint {
    enum class ReferenceType {
      Absolute,
      Relative
    };

    Affine affine;
    std::optional<double> elbow;
    ReferenceType reference_type;


    //! Dynamic Waypoint: Relative velocity factor
    double velocity_rel{1.0};

    //! Dynamic Waypoint: Use maximal dynamics of the robot independent on other parameters
    bool max_dynamics{false};

    //! Zero velocity Waypoint: Stop to zero velocity
    bool zero_velocity{false};

    //! Dynamic Waypoint: Minimum time to get to next waypoint
    std::optional<double> minimum_time;

    //! Path Waypoint: Maximum distance for blending.
    double blend_max_distance{0.0};


    explicit Waypoint() : affine(), reference_type(ReferenceType::Absolute) {}

    explicit Waypoint(const Affine &affine, ReferenceType reference_type = ReferenceType::Absolute)
        : affine(affine), reference_type(reference_type) {}

    explicit Waypoint(const Affine &affine, double elbow, ReferenceType reference_type = ReferenceType::Absolute)
        : affine(affine), reference_type(reference_type), elbow(elbow) {}

    explicit Waypoint(bool zero_velocity)
        : affine(), reference_type(ReferenceType::Relative), zero_velocity(zero_velocity) {}

    explicit Waypoint(double minimum_time)
        : affine(), reference_type(ReferenceType::Relative), minimum_time(minimum_time) {}

    explicit Waypoint(const Affine &affine, ReferenceType reference_type, double velocity_rel)
        : affine(affine), reference_type(reference_type), velocity_rel(velocity_rel) {}

    explicit Waypoint(const Affine &affine, double elbow, ReferenceType reference_type, double velocity_rel)
        : affine(affine), elbow(elbow), reference_type(reference_type), velocity_rel(velocity_rel) {}

    explicit Waypoint(const Affine &affine, std::optional<double> elbow, double blend_max_distance)
        : affine(affine), elbow(elbow), blend_max_distance(blend_max_distance),
          reference_type(ReferenceType::Absolute) {}


    Affine getTargetAffine(const Affine &frame, const Affine &old_affine) const {
      if (reference_type == ReferenceType::Absolute)
        return affine * frame.inverse();
      return old_affine * affine * frame.inverse();
    }

    Vector7d getTargetVector(const Affine &old_affine, double old_elbow) const {
      return getTargetVector(Affine::Identity(), old_affine, old_elbow);
    }

    Vector7d getTargetVector(const Affine &frame, const Affine &old_affine, double old_elbow,
                             const Affine &base_frame = Affine::Identity()) const {
      double new_elbow;
      if (reference_type == ReferenceType::Relative) {
        new_elbow = elbow.value_or(0.0) + old_elbow;
      } else {
        new_elbow = elbow.value_or(old_elbow);
      }
      return RobotPose(base_frame * getTargetAffine(frame, old_affine), new_elbow).vector_repr();
    }
  };

} // namespace franky
