#pragma once

// This one is still under development.

#include <Eigen/Core>

#include <movex/otg/parameter.hpp>


namespace movex {

inline double Power(double v, int e) {
    return std::pow(v, e);
}

inline double Power(double v, double e) {
    return std::pow(v, e);
}

inline double Sqrt(double v) {
    return std::sqrt(v);
}

template<size_t DOFs>
class Ruckig {
    InputParameter<DOFs> current_input;

    enum class ProfileType {
        UP_1, UP_2, UP_3, UP_4, UP_5, UP_6, UP_7, UP_8,
        DOWN_1, DOWN_2, DOWN_3, DOWN_4, DOWN_5, DOWN_6, DOWN_7, DOWN_8
    };

    struct Profile {
        ProfileType type;
        std::array<double, 7> t, t_sum, j;
        std::array<double, 8> a, v, p;

        void set(double p0, double v0, double a0, std::array<double, 7> j) {
            this->j = j;
            t_sum[0] = t[0];
            a[0] = a0;
            v[0] = v0;
            p[0] = p0;

            for (size_t i = 0; i < 6; i += 1) {
                t_sum[i+1] = t_sum[i] + t[i+1];
            }
            for (size_t i = 0; i < 7; i += 1) {
                a[i+1] = a[i] + t[i] * j[i];
                v[i+1] = v[i] + t[i] * a[i] + 0.5 * std::pow(t[i], 2) * j[i];
                p[i+1] = p[i] + t[i] * v[i] + 0.5 * std::pow(t[i], 2) * a[i] + 1. / 6 * std::pow(t[i], 3) * j[i];
            }
        }
    };

    void time_UDDU_case1(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (Power(a0,2) - 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = aMax/jMax;
        profile.t[3] = (3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) - 12*jMax*(2*aMax*jMax*(p0 - pf) + Power(aMax,2)*(v0 + 2*vMax) - jMax*(Power(v0,2) - 2*Power(vMax,2))))/(24.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = aMax/jMax;
        profile.t[5] = -(aMax/jMax) + vMax/aMax;
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case2(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (6*Power(a0,2)*aMax*jMax - 18*Power(aMax,3)*jMax - 12*aMax*Power(jMax,2)*v0 + Sqrt(6)*Sqrt(Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) + 6*(Power(aMax,4) + 4*aMax*Power(jMax,2)*(-p0 + pf) - 2*Power(aMax,2)*jMax*v0 + 2*Power(jMax,2)*Power(v0,2)))))/(12.*Power(aMax,2)*Power(jMax,2));
        profile.t[2] = aMax/jMax;
        profile.t[3] = 0;
        profile.t[4] = aMax/jMax;
        profile.t[5] = (-18*Power(aMax,3)*jMax + Sqrt(6)*Sqrt(Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) + 6*(Power(aMax,4) + 4*aMax*Power(jMax,2)*(-p0 + pf) - 2*Power(aMax,2)*jMax*v0 + 2*Power(jMax,2)*Power(v0,2)))))/(12.*Power(aMax,2)*Power(jMax,2));
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case3(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-2*a0*jMax + Sqrt(2)*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*(-v0 + vMax))))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*(-v0 + vMax)))/(Sqrt(2)*Power(jMax,2));
        profile.t[3] = (-4*Power(a0,3)*aMax*jMax + 12*a0*aMax*Power(jMax,2)*v0 + 3*Sqrt(2)*Power(a0,2)*aMax*Sqrt(Power(jMax,2)*(Power(a0,2) - 2*jMax*v0 + 2*jMax*vMax)) - 6*jMax*(2*aMax*Power(jMax,2)*(p0 - pf) + Power(aMax,2)*jMax*vMax + Power(jMax,2)*Power(vMax,2) + Sqrt(2)*aMax*(v0 + vMax)*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*(-v0 + vMax)))))/(12.*aMax*Power(jMax,3)*vMax);
        profile.t[4] = aMax/jMax;
        profile.t[5] = -(aMax/jMax) + vMax/aMax;
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case4(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (Power(a0,2) - 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = aMax/jMax;
        profile.t[3] = (3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) - 12*jMax*(2*aMax*jMax*(p0 - pf) - 2*aMax*Sqrt(jMax)*Power(vMax,1.5) + Power(aMax,2)*(v0 + vMax) + jMax*(-Power(v0,2) + Power(vMax,2))))/(24.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = -(Sqrt(vMax)/Sqrt(jMax));
        profile.t[5] = 0;
        profile.t[6] = -(Sqrt(vMax)/Sqrt(jMax));

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case5(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-2*a0*jMax + Sqrt(2)*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*(-v0 + vMax))))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*(-v0 + vMax)))/(Sqrt(2)*Power(jMax,2));
        profile.t[3] = (-4*Power(a0,3)*jMax + 12*a0*Power(jMax,2)*v0 + 3*Sqrt(2)*Power(a0,2)*Sqrt(Power(jMax,2)*(Power(a0,2) - 2*jMax*v0 + 2*jMax*vMax)) - 6*(2*Power(jMax,3)*(p0 - pf) + 2*Power(jMax,2.5)*Power(vMax,1.5) + Sqrt(2)*jMax*(v0 + vMax)*Sqrt(Power(jMax,2)*(Power(a0,2) - 2*jMax*v0 + 2*jMax*vMax))))/(12.*Power(jMax,3)*vMax);
        profile.t[4] = Sqrt(vMax)/Sqrt(jMax);
        profile.t[5] = 0;
        profile.t[6] = Sqrt(vMax)/Sqrt(jMax);

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case6(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = -0.1;
        profile.t[1] = 0;
        profile.t[2];
        profile.t[3] = 0;
        profile.t[4] = aMax/jMax;
        profile.t[5];
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case7(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1];
        profile.t[2] = aMax/jMax;
        profile.t[3] = 0;
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6] = -0.1;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case8(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        if (v0 < 1e-16 && a0 < 1e-16) {
            profile.t[0] = Power(-p0 + pf,1./3)/(Power(2,1./3)*Power(jMax,1./3));
            profile.t[1] = 0;
            profile.t[2] = Power(-p0 + pf,1./3)/(Power(2,1./3)*Power(jMax,1./3));
            profile.t[3] = 0;
            profile.t[4] = Power(-p0 + pf,1./3)/(Power(2,1./3)*Power(jMax,1./3));
            profile.t[5] = 0;
            profile.t[6] = Power(-p0 + pf,1./3)/(Power(2,1./3)*Power(jMax,1./3));

            profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
            return;
        }

        profile.t[0];
        profile.t[1] = 0;
        profile.t[2];
        profile.t[3] = 0;
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6];

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_DUUD_case1(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (a0 - aMax)/jMax;
        profile.t[1] = (-Power(a0,2) + 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = -(aMax/jMax);
        profile.t[3] = (3*Power(a0,4) - 8*Power(a0,3)*aMax + 6*Power(a0,2)*Power(aMax,2) - 24*aMax*Power(jMax,2)*p0 + 24*aMax*Power(jMax,2)*pf + 12*Power(a0,2)*jMax*v0 - 24*a0*aMax*jMax*v0 + 12*Power(aMax,2)*jMax*v0 + 12*Power(jMax,2)*Power(v0,2) + 24*Power(aMax,2)*jMax*vMax - 24*Power(jMax,2)*Power(vMax,2))/(24.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = -(aMax/jMax);
        profile.t[5] = (Power(aMax,2) + jMax*vMax)/(aMax*jMax);
        profile.t[6] = -(aMax/jMax);

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case2(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (a0 - aMax)/jMax;
        profile.t[1] = (-6*Power(a0,2) + 18*Power(aMax,2) - 12*jMax*v0 - (Sqrt(6)*Sqrt(Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 6*Power(a0,2)*Power(aMax,2) + 6*Power(aMax,4) - 24*aMax*Power(jMax,2)*p0 + 24*aMax*Power(jMax,2)*pf + 12*Power(a0,2)*jMax*v0 - 24*a0*aMax*jMax*v0 + 12*Power(aMax,2)*jMax*v0 + 12*Power(jMax,2)*Power(v0,2))))/(aMax*jMax))/(12.*aMax*jMax);
        profile.t[2] = -(aMax/jMax);
        profile.t[3] = 0;
        profile.t[4] = -(aMax/jMax);
        profile.t[5] = -(-72*Power(aMax,3)*jMax + Sqrt(5184*Power(aMax,6)*Power(jMax,2) + 96*Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 6*Power(a0,2)*Power(aMax,2) - 48*Power(aMax,4) - 24*aMax*Power(jMax,2)*p0 + 24*aMax*Power(jMax,2)*pf + 12*Power(a0,2)*jMax*v0 - 24*a0*aMax*jMax*v0 + 12*Power(aMax,2)*jMax*v0 + 12*Power(jMax,2)*Power(v0,2))))/(48.*Power(aMax,2)*Power(jMax,2));
        profile.t[6] = -(aMax/jMax);

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case3(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (2*a0*jMax - Sqrt(2)*Sqrt(Power(a0,2)*Power(jMax,2) + 2*Power(jMax,3)*v0 - 2*Power(jMax,3)*vMax))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = -(Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*v0 - 2*jMax*vMax))/(Sqrt(2)*Power(jMax,2)));
        profile.t[3] = (-4*Power(a0,3)*aMax - 12*aMax*Power(jMax,2)*p0 + 12*aMax*Power(jMax,2)*pf - 12*a0*aMax*jMax*v0 + 6*Power(aMax,2)*jMax*vMax - 6*Power(jMax,2)*Power(vMax,2) + (3*Sqrt(2)*Power(a0,2)*aMax*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*v0 - 2*jMax*vMax)))/jMax + 6*Sqrt(2)*aMax*v0*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*v0 - 2*jMax*vMax)) + 6*Sqrt(2)*aMax*vMax*Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*v0 - 2*jMax*vMax)))/(12.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = -(aMax/jMax);
        profile.t[5] = (Power(aMax,2) + jMax*vMax)/(aMax*jMax);
        profile.t[6] = -(aMax/jMax);

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case4(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (a0 - aMax)/jMax;
        profile.t[1] = (-Power(a0,2) + 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = -(aMax/jMax);
        profile.t[3];
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6];

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case5(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (2*a0*jMax - Sqrt(2)*Sqrt(Power(a0,2)*Power(jMax,2) + 2*Power(jMax,3)*v0 - 2*Power(jMax,3)*vMax))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = -(Sqrt(Power(jMax,2)*(Power(a0,2) + 2*jMax*v0 - 2*jMax*vMax))/(Sqrt(2)*Power(jMax,2)));
        profile.t[3];
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6];

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case6(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0];
        profile.t[1] = 0;
        profile.t[2];
        profile.t[3] = 0;
        profile.t[4] = -(aMax/jMax);
        profile.t[5];
        profile.t[6] = -(aMax/jMax);

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case7(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0] = (a0 - aMax)/jMax;
        profile.t[1];
        profile.t[2] = -(aMax/jMax);
        profile.t[3] = 0;
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6];

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case8(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        profile.t[0];
        profile.t[1] = 0;
        profile.t[2];
        profile.t[3] = 0;
        profile.t[4];
        profile.t[5] = 0;
        profile.t[6];

        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void calculate_times(Profile& profile, double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        switch (profile.type) {
            case ProfileType::UP_1: {
                time_UDDU_case1(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_2: {
                time_UDDU_case2(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_3: {
                time_UDDU_case3(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_4: {
                time_UDDU_case4(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_5: {
                time_UDDU_case5(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_6: {
                time_UDDU_case6(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_7: {
                time_UDDU_case7(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::UP_8: {
                time_UDDU_case8(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_1: {
                time_DUUD_case1(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_2: {
                time_DUUD_case2(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_3: {
                time_DUUD_case3(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_4: {
                time_DUUD_case4(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_5: {
                time_DUUD_case5(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_6: {
                time_DUUD_case6(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_7: {
                time_DUUD_case7(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
            case ProfileType::DOWN_8: {
                time_DUUD_case8(profile, p0, v0, a0, pf, vMax, aMax, jMax);
            } break;
        }
    }

    ProfileType get_profile_type(double p0, double v0, double a0, double pf, double vMax, double aMax, double jMax) {
        Profile profile;

        // Test all cases

        time_UDDU_case1(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t > 0; })) {
            return ProfileType::UP_1;  // If no max acc0, acc1, vel:
        }

        time_UDDU_case2(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_2;  // If no max vel
        }

        time_UDDU_case3(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_3;  // If no max acc0
        }

        time_UDDU_case4(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_4;  // // If no max acc1
        }

        time_UDDU_case5(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_5;  // If no max acc0, acc1
        }

        time_UDDU_case6(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_6;  // If no max acc0, vel
        }

        time_UDDU_case7(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_7;  // If no max acc1, vel
        }

        time_UDDU_case8(profile, p0, v0, a0, pf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            return ProfileType::UP_8;  // If no limit:
        }

        throw std::runtime_error("Could not find trajectory.");
    }

    bool calculate(const InputParameter<DOFs>& input) {
        current_input = input;

        // Find profile first.
        for (size_t dof = 0; dof < DOFs; dof += 1) {
            profiles[dof].type = get_profile_type(input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], input.target_position[dof], input.max_velocity[dof], input.max_acceleration[dof], input.max_jerk[dof]);
            calculate_times(profiles[dof], input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], input.target_position[dof], input.max_velocity[dof], input.max_acceleration[dof], input.max_jerk[dof]);
            tf = profiles[dof].t_sum[6];
        }

        t = 0.0;

        return true;
    }

    double t, tf;
    std::array<Profile, DOFs> profiles;

public:
    double delta_time;

    explicit Ruckig(double delta_time): delta_time(delta_time) { }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        t += delta_time;

        if (input != current_input && !calculate(input)) {
            return Result::Error;
        }

        auto p = profiles[0];
        auto index_ptr = std::upper_bound(p.t_sum.begin(), p.t_sum.end(), t);
        size_t index = std::distance(p.t_sum.begin(), index_ptr);

        if (t + delta_time >= p.t_sum[6] - 1e-10) {
            output.new_position[0] = input.target_position[0];
            output.new_velocity[0] = input.target_velocity[0];
            output.new_acceleration[0] = input.target_acceleration[0];
            return Result::Finished;
        }

        double t_diff = t;
        if (index > 0) {
            t_diff -= p.t_sum[index - 1];
        }

        output.new_acceleration[0] = p.a[index] + t_diff * p.j[index];
        output.new_velocity[0] = p.v[index] + t_diff * p.a[index] + 0.5 * std::pow(t_diff, 2) * p.j[index];
        output.new_position[0] = p.p[index] + t_diff * p.v[index] + 0.5 * std::pow(t_diff, 2) * p.a[index] + 1. / 6 * std::pow(t_diff, 3) * p.j[index];

        current_input.current_position = output.new_position;
        current_input.current_velocity = output.new_velocity;
        current_input.current_acceleration = output.new_acceleration;
        return Result::Working;
    }
};

} // namespace movex
