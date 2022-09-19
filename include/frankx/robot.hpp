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

#include <affx/affine.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/robot/kinematics.hpp>

#include <frankx/kinematics.hpp>
#include <frankx/motion_generator.hpp>
#include <frankx/motion_impedance_generator.hpp>
#include <frankx/motion_joint_generator.hpp>
#include <frankx/motion_path_generator.hpp>
#include <frankx/motion_waypoint_generator.hpp>


namespace frankx {
    using namespace movex;
    using Affine = affx::Affine;

class Robot: public franka::Robot {
    // Modified DH-parameters: alpha, d, a
    const KinematicChain<7> kinematics = KinematicChain<7>(
        {{{0.0, 0.333, 0.0}, {-M_PI/2, 0.0, 0.0}, {M_PI/2, 0.316, 0.0}, {M_PI/2, 0.0, 0.0825}, {-M_PI/2, 0.384, -0.0825}, {M_PI/2, 0.0, 0.0}, {M_PI/2, 0.0, 0.088}}},
        Affine(0, 0, 0.107, M_PI/4, 0, M_PI)
    );

public:
    //! The robot's hostname / IP address
    std::string fci_ip;

    // Cartesian constraints
    static constexpr double max_translation_velocity {1.7}; // [m/s]
    static constexpr double max_rotation_velocity {2.5}; // [rad/s]
    static constexpr double max_elbow_velocity {2.175}; // [rad/s]
    static constexpr double max_translation_acceleration {13.0}; // [m/s²]
    static constexpr double max_rotation_acceleration {25.0}; // [rad/s²]
    static constexpr double max_elbow_acceleration {10.0}; // [rad/s²]
    static constexpr double max_translation_jerk {6500.0}; // [m/s³]
    static constexpr double max_rotation_jerk {12500.0}; // [rad/s³]
    static constexpr double max_elbow_jerk {5000.0}; // [rad/s³]

    // Joint constraints
    static constexpr std::array<double, 7> max_joint_velocity {{2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}}; // [rad/s]
    static constexpr std::array<double, 7> max_joint_acceleration {{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0}}; // [rad/s²]
    static constexpr std::array<double, 7> max_joint_jerk {{7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0}}; // [rad/s^3]

    static constexpr size_t degrees_of_freedoms {7};
    static constexpr double control_rate {0.001}; // [s]

    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};

    franka::ControllerMode controller_mode {franka::ControllerMode::kJointImpedance};  // kCartesianImpedance wobbles -> setK?

    //! Whether the robots try to continue an interrupted motion due to a libfranka position/velocity/acceleration discontinuity with reduced dynamics.
    bool repeat_on_error {true};

    //! Whether the robot stops if a python error signal is detected.
    bool stop_at_python_signal {true};

    //! Connects to a robot at the given FCI IP address.
    explicit Robot(std::string fci_ip, double dynamic_rel = 1.0, bool repeat_on_error = true, bool stop_at_python_signal = true);

    void setDefaultBehavior();
    void setDynamicRel(double dynamic_rel);

    bool hasErrors();
    bool recoverFromErrors();

    Affine currentPose(const Affine& frame = Affine(), const bool& read_once = true);
    std::array<double, 7> currentJointPositions(const bool& read_once = true);
    Affine forwardKinematics(const std::array<double, 7>& q);
    std::array<double, 7> inverseKinematics(const Affine& target, const std::array<double, 7>& q0);
    
    ::franka::RobotState* asynchronous_state_ptr;
    ::franka::RobotState get_state(const bool& read_once = true);

    bool move(ImpedanceMotion& motion);
    bool move(ImpedanceMotion& motion, MotionData& data);
    bool move(const Affine& frame, ImpedanceMotion& motion);
    bool move(const Affine& frame, ImpedanceMotion& motion, MotionData& data);

    bool move(JointMotion motion);
    bool move(JointMotion motion, MotionData& data);

    bool move(PathMotion motion);
    bool move(PathMotion motion, MotionData& data);
    bool move(const Affine& frame, PathMotion motion);
    bool move(const Affine& frame, PathMotion motion, MotionData& data);

    bool move(WaypointMotion& motion);
    bool move(WaypointMotion& motion, MotionData& data);
    bool move(const Affine& frame, WaypointMotion& motion);
    bool move(const Affine& frame, WaypointMotion& motion, MotionData& data);
};

} // namespace frankx
