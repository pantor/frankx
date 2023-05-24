#pragma once

#include <Eigen/Core>

#include "franky/path/aggregated_path.hpp"


namespace franky {
  template<size_t state_dimensions>
  class LinearPath : public Path<state_dimensions> {
    using Vector = Eigen::Matrix<double, state_dimensions, 1>;

  public:
    LinearPath() : start(Vector::Zero()), end(Vector::Zero()) {}

    LinearPath(const LinearPath<state_dimensions> &linear_path) : start(linear_path.start), end(linear_path.end) {}

    explicit LinearPath(const Vector &start, const Vector &end) : start(start), end(end) {}

    PathStep<state_dimensions> operator()(double s) const override {
      return {
          start + s / this->length() * (end - start),
          (end - start) / this->length(),
          Vector::Zero(),
          Vector::Zero()
      };
    }

    double length() const override {
      return (end - start).norm();
    }

    Vector max_ddq() const override {
      return Vector::Zero();
    }

    Vector max_dddq() const override {
      return Vector::Zero();
    }

    const Vector start, end;
  };
} // namespace franky
