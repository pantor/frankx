#pragma once

#include <Eigen/Core>


namespace franky {
  template<size_t state_dimensions>
  struct PathStep {
    const Eigen::Vector<double, state_dimensions> q;
    const Eigen::Vector<double, state_dimensions> dq;
    const Eigen::Vector<double, state_dimensions> ddq;
    const Eigen::Vector<double, state_dimensions> dddq;
  };

  template<size_t state_dimensions>
  class Path {
    using Vector = Eigen::Matrix<double, state_dimensions, 1>;
  public:
    virtual double length() const = 0;

    virtual Vector max_ddq() const = 0;

    virtual Vector max_dddq() const = 0;

    virtual PathStep<state_dimensions> operator()(double s) const = 0;

  private:
    size_t length_;
  };
} // namespace franky
