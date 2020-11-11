#pragma once

#include <iostream>
#include <iterator>
#include <vector>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <franka/robot.h>

#include <frankx/affine.hpp>
#include <otgx/parameter.hpp>


namespace frankx {

// Print std::arrays to cout
template <class T, std::size_t N>
inline std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr) {
  std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, ", "));
  return o;
}

inline void setVector(Vector7d& vector, const std::array<double, 7>& data) {
  vector = Eigen::Map<const Vector7d>(data.data(), data.size());
}

template <class T = double>
inline std::array<T, 7> VectorCartRotElbow(T cart, T rot, T elbow) {
  return {cart, cart, cart, rot, rot, rot, elbow};
}

inline franka::CartesianPose CartesianPose(const Vector7d& vector, bool include_elbow = true) {
  auto affine = Affine(vector);
  if (include_elbow) {
    return franka::CartesianPose(affine.array(), {vector(6), -1});
  }
  return franka::CartesianPose(affine.array());
}

} // namespace frankx
