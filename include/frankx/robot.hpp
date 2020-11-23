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
#include <frankx/motion_path.hpp>
#include <frankx/motion_waypoint.hpp>

#include <movex/otg/parameter.hpp>


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
    static constexpr std::array<double, 7> max_joint_velocity {{2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}}; // [rad/s]
    static constexpr std::array<double, 7> max_joint_acceleration {{15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0}}; // [rad/s²]

    double velocity_rel {1.0};
    double acceleration_rel {1.0};
    double jerk_rel {1.0};

    static constexpr size_t degrees_of_freedoms {7};
    const double control_rate {0.001}; // [s]

    franka::ControllerMode controller_mode {franka::ControllerMode::kJointImpedance};  // kCartesianImpedance wobbles -> setK?

    bool repeat_on_error {true};
    bool stop_at_python_signal {true};

    /**
     * Connects to a robot at the given FCI IP address.
     */
    explicit Robot(std::string fci_ip, double dynamic_rel = 1.0, bool repeat_on_error = true, bool stop_at_python_signal = true);

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

    bool move(PathMotion motion);
    bool move(PathMotion motion, MotionData& data);
    bool move(const Affine& frame, PathMotion motion);
    bool move(const Affine& frame, PathMotion motion, MotionData& data);

    bool move(WaypointMotion& motion);
    bool move(WaypointMotion& motion, MotionData& data);
    bool move(const Affine& frame, WaypointMotion& motion);
    bool move(const Affine& frame, WaypointMotion& motion, MotionData& data);

private:
    void setInputLimits(movex::InputParameter<7>& input_parameters, const MotionData& data);
    void setInputLimits(movex::InputParameter<7>& input_parameters, const Waypoint& waypoint, const MotionData& data);

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
};

} // namespace frankx
