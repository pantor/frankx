#ifndef FRANKX_WAYPOINT_HPP
#define FRANKX_WAYPOINT_HPP

#include <frankx/geometry.hpp>


class Waypoint {
  Eigen::Affine3d affine {Eigen::Affine3d::Identity()};
  double elbow {0.0};

  std::array<double, 7> velocity {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

public:
  Waypoint() {}
  Waypoint(const Eigen::Affine3d& affine, double elbow): affine(affine), elbow(elbow) {}
  Waypoint(const Eigen::Affine3d& affine, double elbow, const std::array<double, 7>& velocity): affine(affine), elbow(elbow), velocity(velocity) {}

  franka::CartesianPose getTargetPose() {
    return franka::CartesianPose(Array(affine), {elbow, -1});
  }

  std::array<double, 7> getTargetVelocity() {
    return velocity;
  }
};

#endif // FRANKX_WAYPOINT_HPP
