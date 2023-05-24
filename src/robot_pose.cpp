#include "franky/robot_pose.hpp"

#include <optional>
#include <Eigen/Core>
#include <utility>
#include <franka/control_types.h>

#include "franky/types.hpp"


namespace franky {
  RobotPose::RobotPose(const RobotPose &robot_pose) = default;

  RobotPose::RobotPose(Eigen::Affine3d end_effector_pose, std::optional<double> elbow_position)
      : end_effector_pose_(std::move(end_effector_pose)), elbow_position_(elbow_position) {}

  RobotPose::RobotPose(const Vector7d &vector_repr, bool ignore_elbow)
      : RobotPose(
      vector_repr.head<6>(),
      ignore_elbow ? std::optional<double>(std::nullopt) : vector_repr[6]) {}

  RobotPose::RobotPose(const Vector6d &vector_repr, std::optional<double> elbow_position)
      : elbow_position_(elbow_position),
        end_effector_pose_(Affine().fromPositionOrientationScale(
            vector_repr.head<3>(),
            Eigen::AngleAxis(vector_repr.tail<3>().norm(), vector_repr.tail<3>().normalized()),
            Eigen::Vector3d::Ones())) {}

  RobotPose::RobotPose(const franka::CartesianPose franka_pose)
      : RobotPose(Affine(Eigen::Matrix4d::Map(franka_pose.O_T_EE.data())), franka_pose.elbow[0]) {}

  Vector7d RobotPose::vector_repr() const {
    Eigen::AngleAxis<double> orientation(end_effector_pose_.rotation());
    auto rotvec = orientation.axis() * orientation.angle();
    Vector7d result;
    result << end_effector_pose_.translation(), rotvec, elbow_position_.value_or(0.0);
    return result;
  }

  franka::CartesianPose RobotPose::as_franka_pose() const {
    std::array<double, 16> array;
    std::copy(end_effector_pose_.data(), end_effector_pose_.data() + array.size(), array.begin());
    if (elbow_position_.has_value()) {
      return franka::CartesianPose(array, {elbow_position_.value(), -1});
    }
    return {array};
  }

  RobotPose::RobotPose() : end_effector_pose_(Eigen::Affine3d::Identity()), elbow_position_(std::nullopt) {}
}  // namespace franky
