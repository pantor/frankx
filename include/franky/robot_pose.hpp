#pragma once

#include <franka/control_types.h>

#include <franky/types.hpp>


namespace franky {

  class RobotPose {
  public:
    RobotPose() : end_effector_pose_(Eigen::Affine3d::Identity()), elbow_position_(0.0) {}

    RobotPose(const RobotPose &robot_pose_)
        : end_effector_pose_(robot_pose_.end_effector_pose_), elbow_position_(robot_pose_.elbow_position_) {}

    RobotPose(const Eigen::Affine3d &end_effector_pose, double elbow_position)
        : end_effector_pose_(end_effector_pose), elbow_position_(elbow_position) {}

    RobotPose(const Vector7d &vector_repr) : elbow_position_(vector_repr[6]) {
      end_effector_pose_.translation() = vector_repr.head<3>();
      auto rot_vec = vector_repr.segment<3>(3);
      end_effector_pose_.linear() = Eigen::AngleAxis(rot_vec.norm(), rot_vec.normalized()).toRotationMatrix();
    }

    Vector7d vector_repr() {
      Eigen::AngleAxis<double> orientation(end_effector_pose_.rotation());
      auto rotvec = orientation.axis() * orientation.angle();
      Vector7d result;
      result << end_effector_pose_.translation(), rotvec, elbow_position_;
      return result;
    }

    franka::CartesianPose as_franka_pose(bool include_elbow = true) {
      std::array<double, 16> array;
      std::copy(end_effector_pose_.data(), end_effector_pose_.data() + array.size(), array.begin());
      if (include_elbow) {
        return franka::CartesianPose(array, {elbow_position_, -1});
      }
      return franka::CartesianPose(array);
    }

    Eigen::Affine3d end_effector_pose() {
      return end_effector_pose_;
    }

    double elbow_position() {
      return elbow_position_;
    }

  private:
    Eigen::Affine3d end_effector_pose_;
    double elbow_position_;
  };

} // namespace franky
