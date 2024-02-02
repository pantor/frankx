#pragma once

#include <Eigen/Core>

#include "franky/path/path.hpp"

namespace franky {

template<size_t state_dimensions>
class QuarticBlendPath : public Path<state_dimensions> {
  using Vector = Eigen::Matrix<double, state_dimensions, 1>;

 public:
  QuarticBlendPath() : QuarticBlendPath(Vector::Zero(), Vector::Zero(), Vector::Ones(), 0.0, 0.0, 0.0) {}

  QuarticBlendPath(const QuarticBlendPath<state_dimensions> &quartic_blend_path)
      : length_(quartic_blend_path.length_), b_(quartic_blend_path.b_), c_(quartic_blend_path.c_),
        e_(quartic_blend_path.e_), f_(quartic_blend_path.f_), lm_(quartic_blend_path.lm_),
        rm_(quartic_blend_path.rm_) {}

  explicit QuarticBlendPath(
      const Vector &lb, const Vector &lm, const Vector &rm,
      double s_mid, double max_diff, double s_abs_max) : lb_(lb), lm_(lm), rm_(rm) {
    Vector s_abs = ((-16 * max_diff) / (3. * (lm - rm).array())).abs();
    double s_abs_min = std::min<double>({s_abs.minCoeff(), s_abs_max});
    length_ = 2 * s_abs_min;
    b_ = (lm - rm).array() / (16. * std::pow(s_abs_min, 3));
    c_ = (-lm + rm).array() / (4. * std::pow(s_abs_min, 2));
    e_ = lm;
    f_ = lb.array() + lm.array() * (-s_abs_min + s_mid);
  }

  PathStep<state_dimensions> operator()(double s) const override {
    return {
        f_ + s * (e_ + s * (s * (c_ + s * b_))),
        e_ + s * (s * (3 * c_ + s * 4 * b_)),
        s * (6 * c_ + s * 12 * b_),
        6 * c_ + s * 24 * b_
    };
  }

  [[nodiscard]] inline double length() const override {
    return length_;
  }

  Vector max_ddq() const override {
    return (-3 * (lm_ - rm_)) / (4. * this->length() / 2);
  }

  Vector max_dddq() const override {
    return (3 * (lm_ - rm_)) / (2. * std::pow(this->length() / 2, 2));
  }

 private:
  double length_{};
  Vector7d b_, c_, e_, f_;
  const Vector7d lb_, lm_, rm_;
};

}  // namespace franky
