#pragma once

#include <iostream>
#include <iterator>
#include <vector>

#include <Eigen/Core>
#include <franka/robot.h>

#include <movex/affine.hpp>
#include <movex/otg/parameter.hpp>


namespace frankx {

using Vector6d = Eigen::Matrix<double, 6, 1, Eigen::ColMajor>;
using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
using Affine = movex::Affine;

inline void setVector(Vector7d& vector, const std::array<double, 7>& data) {
  vector = Eigen::Map<const Vector7d>(data.data(), data.size());
}

} // namespace frankx
