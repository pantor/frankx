#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <frankx/waypoint.hpp>


class WaypointMotion {

  std::vector<Waypoint>::const_iterator waypoint_iterator;
  Waypoint current_waypoint;

  double time {0.0};

  std::shared_ptr<ReflexxesAPI> rml;
  std::shared_ptr<RMLPositionInputParameters> input_parameters;
  std::shared_ptr<RMLPositionOutputParameters> output_parameters;

  RMLPositionFlags flags;
  int result_value = 0;


public:
  std::vector<Waypoint> waypoints;

  WaypointMotion(const std::vector<Waypoint>& waypoints): waypoints(waypoints) {}
};
