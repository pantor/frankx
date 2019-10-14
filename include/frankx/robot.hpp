#pragma once

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <frankx/motion_waypoint.hpp>


namespace frankx {
  class Robot: public franka::Robot {
    const double max_translation_velocity {1.7}; // [m/s]
    const double max_rotation_velocity {2.5}; // [rad/s]
    const double max_elbow_velocity {2.175}; // [rad/s]
    const double max_translation_acceleration {13.0}; // [m/s²]
    const double max_rotation_acceleration {25.0}; // [rad/s²]
    const double max_elbow_acceleration {10.0}; // [rad/s²]
    const double max_translation_jerk {6500.0}; // [m/s³]
    const double max_rotation_jerk {12500.0}; // [rad/s³]
    const double max_elbow_jerk {5000.0}; // [rad/s³]

    double velocity_rel {0.08};
    double acceleration_rel {0.05};
    double jerk_rel {0.001};


  public:
    /**
     * Connects to a robot at the given FCI IP address.
     */
    Robot(std::string fci_ip);

    void setDefault();

    void move(const WaypointMotion& motion);
  };
}
