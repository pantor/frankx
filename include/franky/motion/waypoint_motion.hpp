#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include "franky/robot_pose.hpp"
#include "franky/waypoint.hpp"
#include "franky/robot.hpp"
#include "franky/util.hpp"
#include "franky/motion/motion.hpp"

namespace franky {

/**
* A motion following multiple waypoints (with intermediate zero velocity) in a time-optimal way.
* Works with arbitrary initial conditions.
*/
class WaypointMotion : public Motion<franka::CartesianPose> {
 public:
  struct Params {
    Affine frame{Affine::Identity()};
    double velocity_rel{1.0}, acceleration_rel{1.0}, jerk_rel{1.0};
    bool max_dynamics{false};
    bool return_when_finished{true};
  };

  explicit WaypointMotion(std::vector<Waypoint> waypoints);

  explicit WaypointMotion(std::vector<Waypoint> waypoints, Params params);

 protected:
  void initImpl(const franka::RobotState &robot_state, double time) override;

  franka::CartesianPose
  nextCommandImpl(const franka::RobotState &robot_state, franka::Duration time_step, double time) override;

 private:
  std::vector<Waypoint> waypoints_;
  Params params_;

  RobotPose target_robot_pose_;

  Affine ref_frame_;

  ruckig::Ruckig<7> trajectory_generator_{Robot::control_rate};
  ruckig::InputParameter<7> input_para_;
  ruckig::OutputParameter<7> output_para_;

  std::vector<Waypoint>::iterator waypoint_iterator_;
  bool waypoint_has_elbow_{false};

  constexpr static size_t cooldown_iterations_{5};
  size_t current_cooldown_iteration_{0};

  void setNewWaypoint(const franka::RobotState &robot_state, const Waypoint &new_waypoint);

  static inline std::array<double, 7> vec_cart_rot_elbow(double cart, double rot, double elbow) {
    return {cart, cart, cart, rot, rot, rot, elbow};
  }

  [[nodiscard]] std::tuple<std::array<double, 7>, std::array<double, 7>, std::array<double, 7>>
  getInputLimits(const Waypoint &waypoint) const;

  void setInputLimits(ruckig::InputParameter<7> &input_parameters, const Waypoint &waypoint) const;
};

}  // namespace franky
