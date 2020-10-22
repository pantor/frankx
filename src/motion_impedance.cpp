#include <frankx/motion_impedance.hpp>


namespace frankx {

void ImpedanceMotion::setTarget(Affine new_target) {
    target = new_target;
}

bool ImpedanceMotion::isActive() const {
    return is_active;
}

franka::CartesianPose ImpedanceMotion::operator()(const franka::RobotState& robot_state, franka::Duration period) {

}

void ImpedanceMotion::update(Robot* robot, const Affine& frame, const MotionData& motion_data) {
    this->robot = robot;
    this->frame = frame;
    this->motion_data = motion_data;
}

} // namespace frankx
