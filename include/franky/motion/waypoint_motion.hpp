#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/util.hpp"
#include "franky/motion/motion.hpp"

namespace franky {

template<typename TargetType>
struct Waypoint {
  TargetType target;

  ReferenceType reference_type{ReferenceType::Absolute};

  //! Dynamic Waypoint: Relative velocity factor
  double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};

  //! Dynamic Waypoint: Use maximal dynamics of the robot independent on other parameters
  bool max_dynamics{false};

  //! Dynamic Waypoint: Minimum time to get to next waypoint
  std::optional<double> minimum_time{std::nullopt};
};

/**
* A motion following multiple waypoints (with intermediate zero velocity) in a time-optimal way.
* Works with arbitrary initial conditions.
*/
template<typename ControlSignalType, typename TargetType>
class WaypointMotion : public Motion<ControlSignalType> {
 public:
  struct Params {
    double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};
    bool max_dynamics{false};
    bool return_when_finished{true};
  };

  explicit WaypointMotion(std::vector<Waypoint<TargetType>> waypoints, Params params)
      : waypoints_(std::move(waypoints)), params_(std::move(params)), prev_result_() {}

 protected:
  void initImpl(const franka::RobotState &robot_state,
                const std::optional<ControlSignalType> &previous_command) override {
    current_cooldown_iteration_ = 0;

    initWaypointMotion(robot_state, previous_command, input_para_);

    waypoint_iterator_ = waypoints_.begin();
    if (waypoint_iterator_ != waypoints_.end()) {
      setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_para_);
      setInputLimits(*waypoint_iterator_);
      prev_result_ = ruckig::Result::Working;
    } else {
      prev_result_ = ruckig::Result::Finished;
    }
  }

  ControlSignalType
  nextCommandImpl(
      const franka::RobotState &robot_state,
      franka::Duration time_step,
      double rel_time,
      double abs_time,
      const std::optional<ControlSignalType> &previous_command) override {
    const uint64_t steps = std::max<uint64_t>(time_step.toMSec(), 1);
    for (size_t i = 0; i < steps; i++) {
      if (prev_result_ == ruckig::Result::Finished) {
        if (waypoint_iterator_ != waypoints_.end())
          ++waypoint_iterator_;
        if (waypoint_iterator_ == waypoints_.end()) {
          auto output_pose = getControlSignal(input_para_);
          // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
          if (!params_.return_when_finished) {
            return output_pose;
          } else if (current_cooldown_iteration_ < cooldown_iterations_) {
            current_cooldown_iteration_ += 1;
            return output_pose;
          }
          return franka::MotionFinished(output_pose);
        } else {
          setNewWaypoint(robot_state, previous_command, *waypoint_iterator_, input_para_);
          setInputLimits(*waypoint_iterator_);
        }
      }
      if (waypoint_iterator_ != waypoints_.end()) {
        prev_result_ = trajectory_generator_.update(input_para_, output_para_);
        if (prev_result_ == ruckig::Result::Error) {
          throw std::runtime_error("Invalid inputs to motion planner.");
        }
        output_para_.pass_to_input(input_para_);
      }
    }

    return getControlSignal(input_para_);
  };

  virtual void initWaypointMotion(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      ruckig::InputParameter<7> &input_parameter) = 0;

  virtual void setNewWaypoint(
      const franka::RobotState &robot_state,
      const std::optional<ControlSignalType> &previous_command,
      const Waypoint<TargetType> &new_waypoint,
      ruckig::InputParameter<7> &input_parameter) = 0;

  [[nodiscard]] virtual std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const = 0;

  [[nodiscard]] virtual ControlSignalType getControlSignal(const ruckig::InputParameter<7> &input_parameter) const = 0;

 private:
  std::vector<Waypoint<TargetType>> waypoints_;
  Params params_;

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::Result prev_result_;
  ruckig::InputParameter<7> input_para_;
  ruckig::OutputParameter<7> output_para_;

  typename std::vector<Waypoint<TargetType>>::iterator waypoint_iterator_;

  constexpr static size_t cooldown_iterations_{5};
  size_t current_cooldown_iteration_{0};

  void setInputLimits(const Waypoint<TargetType> &waypoint) {
    auto robot = this->robot();

    auto [vel_lim, acc_lim, jerk_lim] = getAbsoluteInputLimits();

    if (!waypoint.max_dynamics && !params_.max_dynamics) {
      vel_lim *= waypoint.velocity_rel * params_.velocity_rel * robot->velocity_rel();
      acc_lim *= waypoint.acceleration_rel * params_.acceleration_rel * robot->acceleration_rel();
      jerk_lim *= waypoint.jerk_rel * params_.jerk_rel * robot->jerk_rel();
    }

    input_para_.max_velocity = toStd<7>(vel_lim);
    input_para_.max_acceleration = toStd<7>(acc_lim);
    input_para_.max_jerk = toStd<7>(jerk_lim);

    if (!(waypoint.max_dynamics || params_.max_dynamics) && waypoint.minimum_time.has_value()) {
      input_para_.minimum_duration = waypoint.minimum_time.value();
    }

    if (waypoint.max_dynamics) {
      input_para_.synchronization = ruckig::Synchronization::TimeIfNecessary;
    } else {
      input_para_.synchronization = ruckig::Synchronization::Time;
    }
  }
};

}  // namespace franky
