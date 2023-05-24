#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

namespace franky {
  class Measure {
    using MeasureFunc = std::function<double(const franka::RobotState &, double)>;

  public:
    explicit Measure(MeasureFunc measure_func);

    explicit Measure(double constant);

    inline double operator()(const franka::RobotState &robot_state, double time) const {
      return measure_func_(robot_state, time);
    }

    static Measure ForceX();

    static Measure ForceY();

    static Measure ForceZ();

    static Measure ForceXYNorm();

    static Measure ForceXYZNorm();

    static Measure Time();

  private:
    MeasureFunc measure_func_;
  };
} // namespace franky
