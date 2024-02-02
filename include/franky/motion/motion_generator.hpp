#pragma once

#include <memory>
#include <mutex>

#include <franka/duration.h>
#include <franka/robot_state.h>
#include "ruckig/ruckig.hpp"

#include "franky/types.hpp"

namespace franky {

struct ReactionRecursionException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

class Robot;

template<typename ControlSignalType>
class Motion;

template<typename ControlSignalType>
class MotionGenerator {
 public:
  static constexpr size_t REACTION_RECURSION_LIMIT = 8;

  explicit MotionGenerator(Robot *robot, std::shared_ptr<Motion<ControlSignalType>> initial_motion);

  ControlSignalType operator()(const franka::RobotState &robot_state, franka::Duration period);

  inline void
  registerUpdateCallback(const std::function<void(const franka::RobotState &, franka::Duration, double)> &callback) {
    update_callbacks_.push_back(callback);
  }

  void updateMotion(std::shared_ptr<Motion<ControlSignalType>> new_motion);

  bool has_new_motion();

  void resetTimeUnsafe();

 private:
  std::shared_ptr<Motion<ControlSignalType>> initial_motion_;
  std::shared_ptr<Motion<ControlSignalType>> current_motion_;
  std::shared_ptr<Motion<ControlSignalType>> new_motion_;
  std::vector<std::function<void(const franka::RobotState &, franka::Duration, double)>> update_callbacks_;
  std::mutex new_motion_mutex_;
  std::optional<ControlSignalType> previous_command_;

  double abs_time_{0.0};
  double rel_time_offset_{0.0};
  Robot *robot_;
};

}  // namespace franky
