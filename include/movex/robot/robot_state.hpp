#pragma once


namespace movex {

//! The overall state of the robot.
template<size_t DoFs, bool has_elbow = true, bool has_force_sensor = true>
struct RobotState {
    std::array<double, DoFs> q;
    std::array<double, DoFs> q_d;
    std::array<double, DoFs> dq;

    std::array<double, 16> O_T_EE;
    std::array<double, 16> O_T_EE_c;
    std::array<double, 6> O_dP_EE_c;

    std::array<double, 2> elbow;
    std::array<double, 2> elbow_c;
    std::array<double, 2> elbow_d;

    std::array<double, 6> O_F_ext_hat_K;
};

} // namespace movex
