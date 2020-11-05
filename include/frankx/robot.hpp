#pragma once

#ifdef WITH_PYTHON
    #include <Python.h>
#endif

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <frankx/motion_data.hpp>
#include <frankx/motion_impedance.hpp>
#include <frankx/motion_joint.hpp>
#include <frankx/motion_waypoint.hpp>
#include <otgx/reflexxes.hpp>


namespace frankx {

class JointMotion;
class ImpedanceMotion;
class WaypointMotion;

struct Robot: public franka::Robot {
    std::string fci_ip;

    // Cartesian
    static constexpr double max_translation_velocity {1.7}; // [m/s]
    static constexpr double max_rotation_velocity {2.5}; // [rad/s]
    static constexpr double max_elbow_velocity {2.175}; // [rad/s]
    static constexpr double max_translation_acceleration {13.0}; // [m/s²]
    static constexpr double max_rotation_acceleration {25.0}; // [rad/s²]
    static constexpr double max_elbow_acceleration {10.0}; // [rad/s²]
    static constexpr double max_translation_jerk {6500.0}; // [m/s³]
    static constexpr double max_rotation_jerk {12500.0}; // [rad/s³]
    static constexpr double max_elbow_jerk {5000.0}; // [rad/s³]

    // Joint
    static constexpr double max_joint_1_velocity {2.175}; // [rad/s]
    static constexpr double max_joint_2_velocity {2.175}; // [rad/s]
    static constexpr double max_joint_3_velocity {2.175}; // [rad/s]
    static constexpr double max_joint_4_velocity {2.175}; // [rad/s]
    static constexpr double max_joint_5_velocity {2.610}; // [rad/s]
    static constexpr double max_joint_6_velocity {2.610}; // [rad/s]
    static constexpr double max_joint_7_velocity {2.610}; // [rad/s]

    static constexpr double max_joint_1_acceleration {15.0}; // [rad/s²]
    static constexpr double max_joint_2_acceleration {7.5}; // [rad/s²]
    static constexpr double max_joint_3_acceleration {10.0}; // [rad/s²]
    static constexpr double max_joint_4_acceleration {12.5}; // [rad/s²]
    static constexpr double max_joint_5_acceleration {15.0}; // [rad/s²]
    static constexpr double max_joint_6_acceleration {20.0}; // [rad/s²]
    static constexpr double max_joint_7_acceleration {20.0}; // [rad/s²]

    const int degrees_of_freedoms {7};
    const double control_rate {0.001};

    double velocity_rel {1.0};
    double acceleration_rel {1.0};
    double jerk_rel {1.0};

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

    bool move(ImpedanceMotion& motion);
    bool move(ImpedanceMotion& motion, MotionData& data);
    bool move(const Affine& frame, ImpedanceMotion& motion);
    bool move(const Affine& frame, ImpedanceMotion& motion, MotionData& data);

    bool move(JointMotion motion);
    bool move(JointMotion motion, MotionData& data);
    bool move(const Affine& frame, JointMotion motion);
    bool move(const Affine& frame, JointMotion motion, MotionData& data);

    bool move(WaypointMotion motion);
    bool move(WaypointMotion motion, MotionData& data);
    bool move(const Affine& frame, WaypointMotion motion);
    bool move(const Affine& frame, WaypointMotion motion, MotionData& data, bool repeat_on_error = true);

private:
    void setInputLimits(RMLPositionInputParameters *input_parameters, const MotionData& data);
    void setInputLimits(RMLPositionInputParameters *input_parameters, const Waypoint& waypoint, const MotionData& data);
};

} // namespace frankx
