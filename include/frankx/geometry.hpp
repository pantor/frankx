#ifndef FRANKX_GEOMETRY_HPP
#define FRANKX_GEOMETRY_HPP

#include <iostream>
#include <iterator>
#include <vector>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <franka/robot.h>
#include <ReflexxesAPI.h>


using Vector7d = Eigen::Matrix<double, 7, 1>;


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

inline Eigen::Vector3d EulerAngles(const Eigen::Affine3d& affine) {
  return Eigen::EulerAnglesZYXd::FromRotation<false, false, false>(affine.rotation()).angles();
}

inline Eigen::Vector3d EulerAngles(const Eigen::Affine3d& affine, const Vector7d& old_vector) {
  Eigen::Vector3d euler = Eigen::EulerAnglesZYXd::FromRotation<false, false, false>(affine.rotation()).angles();
  Eigen::Vector3d euler2;
  euler2 << euler[0] - M_PI, M_PI - euler[1], -M_PI + euler[2];

  if (euler2[1] > M_PI) {
    euler2[1] -= 2 * M_PI;
  }
  if (euler2[2] < -M_PI) {
    euler2[2] += 2 * M_PI;
  }

  Eigen::Vector3d old_euler;
  old_euler << old_vector[3], old_vector[4], old_vector[5];

  if ((old_euler - euler).norm() < (old_euler - euler2).norm()) {
    return euler;
  }
  return euler2;
}

inline Vector7d Vector(const Eigen::Affine3d& affine, double elbow) {
  Vector7d result;
  result << affine.translation(), EulerAngles(affine), elbow;
  return result;
}

inline Vector7d Vector(const Eigen::Affine3d& affine, double elbow, const Vector7d& old_vector) {
  Vector7d result;
  result << affine.translation(), EulerAngles(affine, old_vector), elbow;
  return result;
}

inline Vector7d Vector(const std::vector<double>& vector) {
  Vector7d result;
  result << vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6];
  return result;
}

inline Vector7d Vector(RMLVector<double> *rml_vector) {
  Vector7d result;
  result << rml_vector->VecData[0], rml_vector->VecData[1], rml_vector->VecData[2], rml_vector->VecData[3], rml_vector->VecData[4], rml_vector->VecData[5], rml_vector->VecData[6];
  return result;
}

template <class T = double>
inline std::vector<T> VectorCartRotElbow(T cart, T rot, T elbow) {
  return {cart, cart, cart, rot, rot, rot, elbow};
}

inline Eigen::Affine3d Affine(double x, double y, double z, double a, double b, double c) {
  Eigen::EulerAnglesZYXd euler(a, b, c);

  Eigen::Affine3d affine;
  affine = Eigen::Translation<double, 3>(x, y, z) * euler.toRotationMatrix();
  return affine;
}

inline Eigen::Affine3d Affine(const std::array<double, 16>& array) {
  Eigen::Affine3d affine(Eigen::Matrix4d::Map(array.data()));
  return affine;
}

inline std::array<double, 16> Array(const Eigen::Affine3d& affine) {
  std::array<double, 16> array{};
  std::copy( affine.data(), affine.data() + array.size(), array.begin() );
  return array;
}

inline Eigen::Affine3d getAffine(RMLVector<double> *rml_vector) {
  return Affine(rml_vector->VecData[0], rml_vector->VecData[1], rml_vector->VecData[2], rml_vector->VecData[3], rml_vector->VecData[4], rml_vector->VecData[5]);
}

inline franka::CartesianPose getCartesianPose(RMLVector<double> *rml_vector, bool offset = true) {
  auto affine = getAffine(rml_vector);
  const auto offset_affine = Affine(0.0, 0.0, 0.0, M_PI_2, 0.0, M_PI);
  return franka::CartesianPose(Array(affine * offset_affine.inverse()), {rml_vector->VecData[6], -1});
}

inline Vector7d Vector(const franka::CartesianPose& pose, const Vector7d& old_vector, bool offset = true) {
  Eigen::Affine3d affine = Affine(pose.O_T_EE);
  const auto offset_affine = Affine(0.0, 0.0, 0.0, M_PI_2, 0.0, M_PI);
  return Vector(affine * offset_affine, pose.elbow[0], old_vector);
}

#endif // FRANKX_GEOMETRY_HPP
