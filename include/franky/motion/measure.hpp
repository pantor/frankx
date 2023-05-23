#pragma once

#include <functional>
#include <cmath>
#include <memory>
#include <iostream>
#include <franka/robot_state.h>

namespace franky {

  class Condition {
  public:
    using CheckFunc = std::function<bool(const franka::RobotState &, double)>;

    Condition(CheckFunc callback) : check_func_(callback) {}

    //! Check if the condition is fulfilled
    bool operator()(const franka::RobotState &robot_state, double time) const {
      return check_func_(robot_state, time);
    }

  private:
    CheckFunc check_func_;
  };

  Condition operator&&(const Condition &c1, const Condition &c2) {
    return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
      return c1(robot_state, time) && c2(robot_state, time);
    });
  }

  Condition operator||(const Condition &c1, const Condition &c2) {
    return Condition([c1, c2](const franka::RobotState &robot_state, double time) {
      return c1(robot_state, time) || c2(robot_state, time);
    });
  }

  Condition operator!(const Condition &c) {
    return Condition([c](const franka::RobotState &robot_state, double time) {
      return !c(robot_state, time);
    });
  }


  class Measure {
    using MeasureFunc = std::function<double(const franka::RobotState &, double)>;

  public:
    explicit Measure(MeasureFunc measure_func) : measure_func_(measure_func) {}

    Measure(double constant) : measure_func_([constant](const franka::RobotState &robot_state, double time) {
      return constant;
    }) {}

    bool operator()(const franka::RobotState &robot_state, double time) const {
      return measure_func_(robot_state, time);
    }

    static Measure ForceX() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return robot_state.O_F_ext_hat_K[0];
      });
    }

    static Measure ForceY() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return robot_state.O_F_ext_hat_K[1];
      });
    }

    static Measure ForceZ() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return robot_state.O_F_ext_hat_K[2];
      });
    }

    static Measure ForceXYNorm() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2);
      });
    }

    static Measure ForceXYZNorm() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return std::pow(robot_state.O_F_ext_hat_K[0], 2) + std::pow(robot_state.O_F_ext_hat_K[1], 2) +
               std::pow(robot_state.O_F_ext_hat_K[2], 2);
      });
    }

    static Measure Time() {
      return Measure([](const franka::RobotState &robot_state, double time) {
        return time;
      });
    }

  private:
    MeasureFunc measure_func_;
  };

  Condition operator==(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) == m2(robot_state, time);
    });
  }

  Condition operator!=(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) != m2(robot_state, time);
    });
  }

  Condition operator<=(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) <= m2(robot_state, time);
    });
  }

  Condition operator>=(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) >= m2(robot_state, time);
    });
  }

  Condition operator<(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) < m2(robot_state, time);
    });
  }

  Condition operator>(const Measure &m1, const Measure &m2) {
    return Condition([m1, m2](const franka::RobotState &robot_state, double time) {
      return m1(robot_state, time) > m2(robot_state, time);
    });
  }
} // namespace franky
