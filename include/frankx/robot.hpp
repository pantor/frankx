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

#include <movex/otg/parameter.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_impedance.hpp>
#include <movex/motion/motion_joint.hpp>
#include <movex/motion/motion_path.hpp>
#include <movex/motion/motion_waypoint.hpp>
#include <movex/otg/quintic.hpp>
#include <movex/otg/smoothie.hpp>
#include <movex/otg/ruckig.hpp>

#ifdef WITH_REFLEXXES
#include <movex/otg/reflexxes.hpp>
#endif


namespace movex {
    class ImpedanceMotion;
    class JointMotion;
    class PathMotion;
    class WaypointMotion;
}

namespace frankx {
    using namespace movex;

class Robot: public franka::Robot {
    void setInputLimits(movex::InputParameter<7>& input_parameters, const MotionData& data);
    void setInputLimits(movex::InputParameter<7>& input_parameters, const movex::Waypoint& waypoint, const MotionData& data);

    inline franka::CartesianPose CartesianPose(const Vector7d& vector, bool include_elbow = true) {
        auto affine = Affine(vector);
        if (include_elbow) {
            return franka::CartesianPose(affine.array(), {vector(6), -1});
        }
        return franka::CartesianPose(affine.array());
    }

    template <class T = double>
    inline std::array<T, 7> VectorCartRotElbow(T cart, T rot, T elbow) {
        return {cart, cart, cart, rot, rot, rot, elbow};
    }

    inline void setCartRotElbowVector(Vector7d& vector, double cart, double rot, double elbow) {
        const std::array<double, 7> data = VectorCartRotElbow(cart, rot, elbow);
        vector = Eigen::Map<const Vector7d>(data.data(), data.size());
    }

    movex::RobotState<7> convertState(const franka::RobotState& franka) {
        movex::RobotState<7> movex;
        movex.q = franka.q;
        movex.q_d = franka.q_d;
        movex.dq = franka.dq;

        movex.O_T_EE = franka.O_T_EE;
        movex.O_T_EE_c = franka.O_T_EE_c;
        movex.O_dP_EE_c = franka.O_dP_EE_c;

        movex.elbow = franka.elbow;
        movex.elbow_c = franka.elbow_c;
        movex.elbow_d = franka.elbow_d;

        movex.O_F_ext_hat_K = franka.O_F_ext_hat_K;
        return movex;
    }

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

    double velocity_rel {1.0};
    double acceleration_rel {1.0};
    double jerk_rel {1.0};

    static constexpr size_t degrees_of_freedoms {7};
    static constexpr double control_rate {0.001}; // [s]

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

    Affine currentPose(const Affine& frame = Affine());
    // Affine forwardKinematics(const std::array<double, 7>& q);
    // std::array<double, 7> inverseKinematics(const Affine& target, const Affine& frame = Affine());

    bool move(ImpedanceMotion& motion);
    bool move(ImpedanceMotion& motion, MotionData& data);
    bool move(const Affine& frame, ImpedanceMotion& motion);
    bool move(const Affine& frame, ImpedanceMotion& motion, MotionData& data);

    bool move(JointMotion motion);
    bool move(JointMotion motion, MotionData& data);
    bool move(const Affine& frame, JointMotion motion);
    bool move(const Affine& frame, JointMotion motion, MotionData& data);

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
