#pragma once

#include <optional>
#include <Eigen/Core>
#include <franka/control_types.h>

#include "franky/types.hpp"


namespace franky {

  class RobotPose {
  public:
    RobotPose() : end_effector_pose_(Eigen::Affine3d::Identity()), elbow_position_(std::nullopt) {}

    RobotPose(const RobotPose &robot_pose)
        : end_effector_pose_(robot_pose.end_effector_pose_), elbow_position_(robot_pose.elbow_position_) {}

    explicit RobotPose(const Eigen::Affine3d &end_effector_pose, std::optional<double> elbow_position = std::nullopt)
        : end_effector_pose_(end_effector_pose), elbow_position_(elbow_position) {}

    explicit RobotPose(const Vector7d &vector_repr, bool ignore_elbow = false)
        : RobotPose(
        vector_repr.head<6>(),
        ignore_elbow ? std::optional<double>(std::nullopt) : vector_repr[6]) {}

    explicit RobotPose(const Vector6d &vector_repr, std::optional<double> elbow_position = std::nullopt)
        : elbow_position_(elbow_position),
          end_effector_pose_(Affine().fromPositionOrientationScale(
              vector_repr.head<3>(),
              Eigen::AngleAxis(vector_repr.tail<3>().norm(), vector_repr.tail<3>().normalized()),
              Eigen::Vector3d::Ones())) {}

    explicit RobotPose(const franka::CartesianPose franka_pose)
        : RobotPose(Affine(Eigen::Matrix4d::Map(franka_pose.O_T_EE.data())), franka_pose.elbow[0]) {}

    Vector7d vector_repr() const {
      Eigen::AngleAxis<double> orientation(end_effector_pose_.rotation());
      auto rotvec = orientation.axis() * orientation.angle();
      Vector7d result;
      result << end_effector_pose_.translation(), rotvec, elbow_position_;
      return result;
    }

    franka::CartesianPose as_franka_pose() const {
      std::array<double, 16> array;
      std::copy(end_effector_pose_.data(), end_effector_pose_.data() + array.size(), array.begin());
      if (elbow_position_.has_value()) {
        return franka::CartesianPose(array, {elbow_position_.value(), -1});
      }
      return franka::CartesianPose(array);
    }

    RobotPose left_transform(const Affine &transform) const {
      return RobotPose(transform * end_effector_pose_, elbow_position_);
    }

    RobotPose right_transform(const Affine &transform) const {
      return RobotPose(end_effector_pose_ * transform, elbow_position_);
    }

    RobotPose with_elbow_position(const std::optional<double> elbow_position) const {
      return RobotPose(end_effector_pose_, elbow_position);
    }

    Affine end_effector_pose() const {
      return end_effector_pose_;
    }

    std::optional<double> elbow_position() const {
      return elbow_position_;
    }

  private:
    Affine end_effector_pose_;
    std::optional<double> elbow_position_;
  };

  RobotPose operator*(const RobotPose &robot_pose, const Affine &right_transform) {
    return robot_pose.right_transform(right_transform);
  }

  RobotPose operator*(const Affine &left_transform, const RobotPose &robot_pose) {
    return robot_pose.left_transform(left_transform);
  }
} // namespace franky
