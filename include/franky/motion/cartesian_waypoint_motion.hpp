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

struct CartesianWaypoint : public Waypoint {
  RobotPose robot_pose;
  ReferenceType reference_type{ReferenceType::Absolute};
};

class CartesianWaypointMotion : public WaypointMotion<franka::CartesianPose, CartesianWaypoint> {
 public:
  struct Params : WaypointMotion<franka::CartesianPose, CartesianWaypoint>::Params {
    Affine frame{Affine::Identity()};
  };

  explicit CartesianWaypointMotion(const std::vector<CartesianWaypoint> &waypoints);

  explicit CartesianWaypointMotion(const std::vector<CartesianWaypoint> &waypoints, Params params);

 protected:

  void initWaypointMotion(const franka::RobotState &robot_state, ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(const franka::RobotState &robot_state,
                      const CartesianWaypoint &new_waypoint,
                      ruckig::InputParameter<7> &input_parameter) override;

  std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  franka::CartesianPose getControlSignal(const ruckig::InputParameter<7> &input_parameter) const override;

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
