#pragma once

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


namespace franky {
  template<size_t dims>
  using Vector = Eigen::Matrix<double, dims, 1>;

  using Vector6d = Vector<6>;
  using Vector7d = Vector<7>;

  using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemXYZ>;

  using Affine = Eigen::Affine3d;
}