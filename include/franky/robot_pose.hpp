#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"


namespace franky {
  class RobotPose {
  public:
    RobotPose() : end_effector_pose_(Eigen::Affine3d::Identity()), elbow_position_(std::nullopt) {}

    RobotPose(const RobotPose &robot_pose);

    explicit RobotPose(const Eigen::Affine3d &end_effector_pose, std::optional<double> elbow_position = std::nullopt);

    explicit RobotPose(const Vector7d &vector_repr, bool ignore_elbow = false);

    explicit RobotPose(const Vector6d &vector_repr, std::optional<double> elbow_position = std::nullopt);

    explicit RobotPose(const franka::CartesianPose franka_pose);

    Vector7d vector_repr() const;

    franka::CartesianPose as_franka_pose() const;

    inline RobotPose left_transform(const Affine &transform) const {
      return RobotPose(transform * end_effector_pose_, elbow_position_);
    }

    inline RobotPose right_transform(const Affine &transform) const {
      return RobotPose(end_effector_pose_ * transform, elbow_position_);
    }

    inline RobotPose with_elbow_position(const std::optional<double> elbow_position) const {
      return RobotPose(end_effector_pose_, elbow_position);
    }

    inline Affine end_effector_pose() const {
      return end_effector_pose_;
    }

    inline std::optional<double> elbow_position() const {
      return elbow_position_;
    }

  private:
    Affine end_effector_pose_;
    std::optional<double> elbow_position_;
  };

  inline RobotPose operator*(const RobotPose &robot_pose, const Affine &right_transform) {
    return robot_pose.right_transform(right_transform);
  }

  inline RobotPose operator*(const Affine &left_transform, const RobotPose &robot_pose) {
    return robot_pose.left_transform(left_transform);
  }
} // namespace franky
