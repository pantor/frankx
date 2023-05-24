#pragma once

#include <Eigen/Core>

#include "franky/path/aggregated_path.hpp"


namespace franky {
  template<size_t state_dimensions>
  class LinearPath : public Path<state_dimensions> {
    using Vector = Eigen::Matrix<double, state_dimensions, 1>;

  public:
    LinearPath() : start_(Vector::Zero()), end_(Vector::Zero()) {}

    LinearPath(const LinearPath<state_dimensions> &linear_path) : start_(linear_path.start_), end_(linear_path.end_) {}

    explicit LinearPath(const Vector &start, const Vector &end) : start_(start), end_(end) {}

    PathStep<state_dimensions> operator()(double s) const override {
      return {
          start_ + s / this->length() * (end_ - start_),
          (end_ - start_) / this->length(),
          Vector::Zero(),
          Vector::Zero()
      };
    }

    [[nodiscard]] inline double length() const override {
      return (end_ - start_).norm();
    }

    inline Vector max_ddq() const override {
      return Vector::Zero();
    }

    inline Vector max_dddq() const override {
      return Vector::Zero();
    }

    inline Vector start() const {
      return start_;
    }

    inline Vector end() const {
      return end_;
    }

  private:
    Vector start_, end_;
  };
}  // namespace franky
