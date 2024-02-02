#pragma once

#include <string>

namespace franky {

class RelativeDynamicsFactor {
 public:
  RelativeDynamicsFactor();;

  RelativeDynamicsFactor(double value);

  RelativeDynamicsFactor(double velocity, double acceleration, double jerk);;

  static inline RelativeDynamicsFactor MAX_DYNAMICS() {
    return {1.0, 1.0, 1.0, true};
  }

  [[nodiscard]] inline double velocity() const {
    return velocity_;
  }

  [[nodiscard]] inline double acceleration() const {
    return acceleration_;
  }

  [[nodiscard]] inline double jerk() const {
    return jerk_;
  }

  [[nodiscard]] inline bool max_dynamics() const {
    return max_dynamics_;
  }

  RelativeDynamicsFactor operator*(const RelativeDynamicsFactor &other) const;

 private:
  static double checkInBounds(double value, const std::string &name);

  RelativeDynamicsFactor(double velocity, double acceleration, double jerk, bool max_dynamics);

  double velocity_, acceleration_, jerk_;
  bool max_dynamics_;
};

}  // namespace franky
