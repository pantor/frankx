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

struct JointWaypoint : public Waypoint {
  Vector7d target;
};

class JointWaypointMotion : public WaypointMotion<franka::JointPositions, JointWaypoint> {
 public:
  explicit JointWaypointMotion(const std::vector<JointWaypoint> &waypoints);

  explicit JointWaypointMotion(const std::vector<JointWaypoint> &waypoints, Params params);

 protected:

  void initWaypointMotion(const franka::RobotState &robot_state, ruckig::InputParameter<7> &input_parameter) override;

  void setNewWaypoint(const franka::RobotState &robot_state,
                      const JointWaypoint &new_waypoint,
                      ruckig::InputParameter<7> &input_parameter) override;

  std::tuple<Vector7d, Vector7d, Vector7d> getAbsoluteInputLimits() const override;

  franka::JointPositions getControlSignal(const ruckig::InputParameter<7> &input_parameter) const override;
};

}  // namespace franky
