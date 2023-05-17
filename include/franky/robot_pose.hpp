#pragma once

#include <franka/control_types.h>

#include <franky/types.hpp>


namespace franky {

  class RobotPose {
  public:
    RobotPose() : end_effector_pose(Eigen::Affine3d::Identity()), elbow_position(0.0) {}

    RobotPose(const RobotPose &robot_pose_)
        : end_effector_pose(robot_pose_.end_effector_pose), elbow_position(robot_pose_.elbow_position) {}

    RobotPose(const Eigen::Affine3d &end_effector_pose, double elbow_position)
        : end_effector_pose(end_effector_pose), elbow_position(elbow_position) {}

    RobotPose(const Vector7d &vector_repr)
        : elbow_position(vector_repr[6]),
          end_effector_pose(Affine().fromPositionOrientationScale(
              vector_repr.head<3>(),
              Eigen::AngleAxis(vector_repr.segment<3>(3).norm(), vector_repr.segment<3>(3).normalized()),
              Eigen::Vector3d::Ones())) {}

    Vector7d vector_repr() {
      Eigen::AngleAxis<double> orientation(end_effector_pose.rotation());
      auto rotvec = orientation.axis() * orientation.angle();
      Vector7d result;
      result << end_effector_pose.translation(), rotvec, elbow_position;
      return result;
    }

    franka::CartesianPose as_franka_pose(bool include_elbow = true) {
      std::array<double, 16> array;
      std::copy(end_effector_pose.data(), end_effector_pose.data() + array.size(), array.begin());
      if (include_elbow) {
        return franka::CartesianPose(array, {elbow_position, -1});
      }
      return franka::CartesianPose(array);
    }

    RobotPose left_transform(const Affine &transform) const {
      return RobotPose(transform * end_effector_pose, elbow_position);
    }

    RobotPose right_transform(const Affine &transform) const {
      return RobotPose(end_effector_pose * transform, elbow_position);
    }

    const Affine end_effector_pose;
    const double elbow_position;
  };

  RobotPose operator*(const RobotPose &robot_pose, const Affine &right_transform) {
    return robot_pose.right_transform(right_transform);
  }

  RobotPose operator*(const Affine &left_transform, const RobotPose &robot_pose) {
    return robot_pose.left_transform(left_transform);
  }
} // namespace franky
