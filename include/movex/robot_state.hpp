#pragma once


namespace movex {

class RobotState {
    std::array<double, 7> q;
    std::array<double, 7> q_d;
    std::array<double, 7> dq;
    
    std::array<double, 16> O_T_EE;
    std::array<double, 16> O_T_EE_c;
    std::array<double, 6> O_dP_EE_c;

    double elbow;
    double elbow_c;
    double eblow_d;
    
    std::array<double, 6> O_F_ext_hat_K;
};

} // namespace movex
