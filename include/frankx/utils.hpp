#pragma once

#include <iostream>
#include <iterator>
#include <vector>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <franka/robot.h>
#include <ReflexxesAPI.h>

#include <frankx/affine.hpp>


namespace frankx {

// Print std::arrays to cout
template <class T, std::size_t N>
inline std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr) {
  std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, ", "));
  return o;
}

// Print rml vector to cout
template <class T>
inline std::ostream& operator<<(std::ostream& o, const RMLVector<T> *rml_vector) {
  for (int i = 0; i < 7; i++) {
    o << rml_vector->VecData[i] << ", ";
  }
  o << std::endl;
  return o;
}

template <class T = double>
inline void setVector(RMLVector<T> *rml_vector, const std::vector<T>& vector) {
  for (int i = 0; i < 7; i++) {
    rml_vector->VecData[i] = vector[i];
  }
  // std::copy(vector.begin(), vector.end(), rml_vector->VecData);
}

template <class T = double>
inline void setVector(RMLVector<T> *rml_vector, const std::array<T, 7>& vector) {
  for (int i = 0; i < 7; i++) {
    rml_vector->VecData[i] = vector[i];
  }
  // std::copy(vector.begin(), vector.end(), rml_vector->VecData);
}

inline void setVector(RMLVector<double> *rml_vector, const Vector7d& vector) {
  std::copy(vector.data(), vector.data() + vector.size(), rml_vector->VecData);
}

inline void setZero(RMLVector<double> *rml_vector) {
  setVector(rml_vector, std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

inline Vector7d Vector(const std::vector<double>& vector) {
  Vector7d result;
  result << vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6];
  return result;
}

inline Vector7d Vector(RMLVector<double> *rml_vector) {
  double* vector = rml_vector->VecData;
  Vector7d result;
  result << vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6];
  return result;
}

template <class T = double>
inline std::vector<T> VectorCartRotElbow(T cart, T rot, T elbow) {
  return {cart, cart, cart, rot, rot, rot, elbow};
}

inline franka::CartesianPose CartesianPose(RMLVector<double> *rml_vector, bool include_elbow = true) {
  auto affine = Affine(rml_vector);
  if (include_elbow) {
    return franka::CartesianPose(affine.array(), {rml_vector->VecData[6], -1});
  }
  return franka::CartesianPose(affine.array());
}

} // namespace frankx
