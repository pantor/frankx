#pragma once

#include <optional>

#include <Eigen/Geometry>

#include "franky/robot_pose.hpp"
#include "franky/types.hpp"


namespace franky {

  struct Waypoint {
    enum class ReferenceType {
      Absolute,
      Relative
    };

    RobotPose robot_pose;
    ReferenceType reference_type{ReferenceType::Absolute};

    //! Dynamic Waypoint: Relative velocity factor
    double velocity_rel{1.0};

    //! Dynamic Waypoint: Use maximal dynamics of the robot independent on other parameters
    bool max_dynamics{false};

    //! Dynamic Waypoint: Minimum time to get to next waypoint
    std::optional<double> minimum_time;

    //! Path Waypoint: Maximum distance for blending.
    double blend_max_distance{0.0};

    RobotPose getTargetRobotPose(const RobotPose &old_robot_pose) const {
      std::optional<double> new_elbow;
      if (robot_pose.elbow_position().has_value() && reference_type == ReferenceType::Relative) {
        if (!old_robot_pose.elbow_position().has_value())
          new_elbow = robot_pose.elbow_position();
        else {
          new_elbow = robot_pose.elbow_position().value() + old_robot_pose.elbow_position().value();
        }
      } else {
        new_elbow = robot_pose.elbow_position();
      }
      RobotPose new_robot_pose = robot_pose.with_elbow_position(new_elbow);
      if (reference_type == ReferenceType::Absolute)
        return new_robot_pose;
      return old_robot_pose.end_effector_pose() * new_robot_pose;
    };
  };

} // namespace franky
