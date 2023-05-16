#pragma once

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


namespace franky {
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

  using Affine = Eigen::Affine3d;
}