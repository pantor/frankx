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

    PathStep<state_dimensions> operator()(double s) const {
      return {
          start + s / this->length() * (end - start),
          (end - start) / this->length(),
          Vector::Zero(),
          Vector::Zero()
      };
    }

    virtual double length() const {
      return (end - start).norm();
    }

    virtual Vector max_ddq() const {
      return Vector::Zero();
    }

    virtual Vector max_dddq() const {
      return Vector::Zero();
    }

    const Vector start, end;
  };
} // namespace franky
