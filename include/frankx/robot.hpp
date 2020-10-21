#pragma once

#ifdef WITH_PYTHON
    #include <Python.h>
#endif

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <frankx/motion_data.hpp>
#include <frankx/motion_joint.hpp>
#include <frankx/motion_waypoint.hpp>


namespace frankx {

struct Robot: public franka::Robot {
    std::string fci_ip;

    static constexpr double max_translation_velocity {1.7}; // [m/s]
    static constexpr double max_rotation_velocity {2.5}; // [rad/s]
    static constexpr double max_elbow_velocity {2.175}; // [rad/s]
    static constexpr double max_translation_acceleration {13.0}; // [m/s²]
    static constexpr double max_rotation_acceleration {25.0}; // [rad/s²]
    static constexpr double max_elbow_acceleration {10.0}; // [rad/s²]
    static constexpr double max_translation_jerk {6500.0}; // [m/s³]
    static constexpr double max_rotation_jerk {12500.0}; // [rad/s³]
    static constexpr double max_elbow_jerk {5000.0}; // [rad/s³]

    double velocity_rel {0.1};
    double acceleration_rel {0.1};
    double jerk_rel {0.01};

    franka::ControllerMode controller_mode {franka::ControllerMode::kJointImpedance};  // kCartesianImpedance wobbles -> setK?

    /**
     * Connects to a robot at the given FCI IP address.
     */
    explicit Robot(std::string fci_ip, double dynamic_rel = 1.0);

    void setDefaultBehavior();
    void setDynamicRel(double dynamic_rel);

    bool hasErrors();
    bool recoverFromErrors();

    Affine currentPose(const Affine& frame = Affine());

    bool move(JointMotion motion);
    bool move(JointMotion motion, MotionData& data);

    bool move(WaypointMotion motion);
    bool move(WaypointMotion motion, MotionData& data);
    bool move(const Affine& frame, WaypointMotion motion);
    bool move(const Affine& frame, WaypointMotion motion, MotionData& data);

private:
    void setInputLimits(RMLPositionInputParameters *input_parameters, const MotionData& data);
    void setInputLimits(RMLPositionInputParameters *input_parameters, const Waypoint& waypoint, const MotionData& data);
};

} // namespace frankx
