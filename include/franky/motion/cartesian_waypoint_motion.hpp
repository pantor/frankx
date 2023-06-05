#pragma once

#include <atomic>
#include <optional>
#include <ruckig/ruckig.hpp>

#include <franka/robot_state.h>

#include "franky/robot_pose.hpp"
#include "franky/robot.hpp"
#include "franky/motion/reference_type.hpp"
#include "franky/util.hpp"
#include "franky/motion/waypoint_motion.hpp"

namespace franky {

class CartesianWaypointMotion : public WaypointMotion<franka::CartesianPose, RobotPose> {
 public:
  struct Params : WaypointMotion<franka::CartesianPose, RobotPose>::Params {
    Affine frame{Affine::Identity()};
    WaypointMotion<franka::CartesianPose, RobotPose>::Params base_params{};
  };

  explicit CartesianWaypointMotion(const std::vector<Waypoint<RobotPose>> &waypoints);

  explicit CartesianWaypointMotion(const std::vector<Waypoint<RobotPose>> &waypoints, Params params);

 protected:

  void initWaypointMotion(const franka::RobotState &robot_state, ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(const franka::RobotState &robot_state,
                      const Waypoint<RobotPose> &new_waypoint,
                      ruckig::InputParameter<7> &input_parameter) override;

  [[nodiscard]] std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  [[nodiscard]] franka::CartesianPose getControlSignal(const ruckig::InputParameter<7> &input_parameter) const override;

 private:
  Params params_;

  RobotPose target_robot_pose_;
  Affine ref_frame_;

  [[nodiscard]] RobotPose getTargetRobotPose(const RobotPose &old_robot_pose) const;

  static inline Vector7d vec_cart_rot_elbow(double cart, double rot, double elbow) {
    return {cart, cart, cart, rot, rot, rot, elbow};
  }
};

}  // namespace franky
