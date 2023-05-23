#pragma once

#include <array>
#include <Eigen/Core>

template<size_t dims>
static inline std::array<double, dims> toStd(const Eigen::Matrix<double, dims, 1>& vector) {
  std::array<double, 7> result;
  Eigen::Matrix<double, dims, 1>::Map(result.data()) = vector;
  return result;
}

template<size_t dims>
static inline Eigen::Matrix<double, dims, 1> toEigen(const std::array<double, dims>& vector) {
  return Eigen::Matrix<double, dims, 1>::Map(vector.data());
}