#ifndef FRANKX_GEOMETRY_HPP
#define FRANKX_GEOMETRY_HPP

#include <iostream>
#include <iterator>
#include <vector>

#include <franka/robot.h>

#include <Eigen/Geometry>
// #include <unsupported/Eigen/EulerAngles>

#include <ReflexxesAPI.h>


using Vector7d = Eigen::Matrix<double, 7, 1>;


// Print std::arrays to cout
template <class T, std::size_t N>
inline std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr) {
  std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, ", "));
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
  for (int i = 0; i < 7; i++) {
    rml_vector->VecData[i] = vector(i);
  }
  // std::copy(vector.data(), vector.data() + vector.size(), rml_vector->VecData);
}

inline void setZero(RMLVector<double> *rml_vector) {
  setVector(rml_vector, std::array<double, 7>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

inline Eigen::Vector3d EulerAngles(const Eigen::Affine3d& affine, bool offset = false) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  if (offset) {
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  }

  Eigen::Vector3d euler = (affine.rotation() * m).eulerAngles(1, 0, 2);

  Eigen::Vector3d result;
  result << euler[0], euler[1], euler[2];
  return result;
}

inline Vector7d Vector(const Eigen::Affine3d& affine, double elbow, bool offset = false) {
  Vector7d result;
  result << affine.translation(), EulerAngles(affine, offset), elbow;
  return result;
}

inline Vector7d Vector(const std::vector<double>& vector) {
  Vector7d result;
  result << vector[0], vector[1], vector[2], vector[3], vector[4], vector[5], vector[6];
  return result;
}

template <class T = double>
inline std::vector<T> VectorCartRotElbow(T cart, T rot, T elbow) {
  return {cart, cart, cart, rot, rot, rot, elbow};
}

inline Eigen::Affine3d Affine(double x, double y, double z, double a, double b, double c, bool offset = false) {
  Eigen::AngleAxisd roll(a, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitch(b, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yaw(c, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> q = roll * pitch * yaw;

  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  if (offset) {
    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  }

  Eigen::Affine3d affine;
  affine = Eigen::Translation<double, 3>(x, y, z) * (q.matrix() * m.inverse());
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

inline Eigen::Affine3d getAffine(RMLVector<double> *rml_vector, bool offset = false) {
  return Affine(rml_vector->VecData[0], rml_vector->VecData[1], rml_vector->VecData[2], rml_vector->VecData[3], rml_vector->VecData[4], rml_vector->VecData[5], offset);
}

inline franka::CartesianPose getCartesianPose(RMLVector<double> *rml_vector, bool offset = false) {
  return franka::CartesianPose(Array(getAffine(rml_vector, offset)), {rml_vector->VecData[6], -1});
}

inline Vector7d Vector(const franka::CartesianPose& pose, bool offset = false) {
  return Vector(Affine(pose.O_T_EE), pose.elbow[0], offset);
}

#endif // FRANKX_GEOMETRY_HPP
