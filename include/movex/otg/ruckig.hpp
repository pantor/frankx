#pragma once

// This one is still under development.

#include <chrono>
#include <complex>

#include <Eigen/Core>

#include <movex/otg/parameter.hpp>


namespace movex {

template<size_t DOFs>
class Ruckig {
    InputParameter<DOFs> current_input;

    enum class ProfileType {
        UP_ACC0_ACC1_VEL, UP_VEL, UP_ACC0, UP_ACC1, UP_ACC0_ACC1, UP_ACC0_VEL, UP_ACC1_VEL, UP_NONE,
        DOWN_ACC0_ACC1_VEL, DOWN_VEL, DOWN_ACC0, DOWN_ACC1, DOWN_ACC0_ACC1, DOWN_ACC0_VEL, DOWN_ACC1_VEL, DOWN_NONE
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

        void reset(double p0, double v0, double a0, double base_jerk) {
            std::array<double, 7> new_j;
            for (size_t step = 0; step < 7; step += 1) {
                if (j[step] > 0.0) {
                    new_j[step] = base_jerk;
                } else if (j[step] < 0.0) {
                    new_j[step] = -base_jerk;
                }
            }
            set(p0, v0, a0, new_j);
        }
    };

    inline static double Power(double v, int e) {
        return std::pow(v, e);
    }

    inline static double Power(double v, double e) {
        return std::pow(v, e);
    }

    inline static double Sqrt(double v) {
        return std::sqrt(v);
    }

    inline static double Abs(double v) {
        return std::abs(v);
    }

    void time_UDDU_case1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (Power(a0,2) - 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = aMax/jMax;
        profile.t[3] = (3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) - 12*jMax*(2*aMax*jMax*(p0 - pf) + Power(aMax,2)*(v0 + vf + 2*vMax) - jMax*(Power(v0,2) + Power(vf,2) - 2*Power(vMax,2))))/(24.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = aMax/jMax;
        profile.t[5] = (-(Power(aMax,2)/jMax) - vf + vMax)/aMax;
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case2(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (6*Power(a0,2)*aMax*jMax - 18*Power(aMax,3)*jMax - 12*aMax*Power(jMax,2)*v0 + Sqrt(6)*Sqrt(Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) + 6*(Power(aMax,4) + 4*aMax*Power(jMax,2)*(-p0 + pf) - 2*Power(aMax,2)*jMax*(v0 + vf) + 2*Power(jMax,2)*(Power(v0,2) + Power(vf,2))))))/(12.*Power(aMax,2)*Power(jMax,2));
        profile.t[2] = aMax/jMax;
        profile.t[3] = 0;
        profile.t[4] = aMax/jMax;
        profile.t[5] = (-18*Power(aMax,3)*jMax - 12*aMax*Power(jMax,2)*vf + Sqrt(6)*Sqrt(Power(aMax,2)*Power(jMax,2)*(3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) + 6*(Power(aMax,4) + 4*aMax*Power(jMax,2)*(-p0 + pf) - 2*Power(aMax,2)*jMax*(v0 + vf) + 2*Power(jMax,2)*(Power(v0,2) + Power(vf,2))))))/(12.*Power(aMax,2)*Power(jMax,2));
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case3(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-2*a0*jMax + Sqrt(2)*Sqrt(Power(a0,2) + 2*jMax*(-v0 + vMax))*Abs(jMax))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = (Sqrt(Power(a0,2)/2. + jMax*(-v0 + vMax))*Abs(jMax))/Power(jMax,2);
        profile.t[3] = (-2*jMax*(2*Power(a0,3)*aMax - 6*a0*aMax*jMax*v0 + 3*jMax*(2*aMax*jMax*(p0 - pf) + Power(aMax,2)*(vf + vMax) + jMax*(-Power(vf,2) + Power(vMax,2)))) + 3*Sqrt(2)*aMax*Sqrt(Power(a0,2) + 2*jMax*(-v0 + vMax))*(Power(a0,2) - 2*jMax*(v0 + vMax))*Abs(jMax))/(12.*aMax*Power(jMax,3)*vMax);
        profile.t[4] = aMax/jMax;
        profile.t[5] = (-(Power(aMax,2)/jMax) - vf + vMax)/aMax;
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case4(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = (Power(a0,2) - 2*Power(aMax,2) - 2*jMax*v0 + 2*jMax*vMax)/(2.*aMax*jMax);
        profile.t[2] = aMax/jMax;
        profile.t[3] = (3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) - 12*jMax*(Power(aMax,2)*(v0 + vMax) + jMax*(-Power(v0,2) + Power(vMax,2)) + 2*aMax*(jMax*(p0 - pf) + Sqrt(jMax)*Sqrt(-vf + vMax)*(vf + vMax))))/(24.*aMax*Power(jMax,2)*vMax);
        profile.t[4] = Sqrt(-vf + vMax)/Sqrt(jMax);
        profile.t[5] = 0;
        profile.t[6] = Sqrt(-vf + vMax)/Sqrt(jMax);

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    bool time_UDDU_case5(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        profile.t[0] = (-2*a0*jMax + Sqrt(2)*Sqrt(Power(a0,2) + 2*jMax*(-v0 + vMax))*Abs(jMax))/(2.*Power(jMax,2));
        profile.t[1] = 0;
        profile.t[2] = (Sqrt(Power(a0,2)/2. + jMax*(-v0 + vMax))*Abs(jMax))/Power(jMax,2);
        profile.t[3] = (-4*jMax*(Power(a0,3) + 3*Power(jMax,2)*(p0 - pf) - 3*a0*jMax*v0 + 3*jMax*Sqrt(jMax)*Sqrt(-vf + vMax)*(vf + vMax)) + 3*Sqrt(2)*Sqrt(Power(a0,2) + 2*jMax*(-v0 + vMax))*(Power(a0,2) - 2*jMax*(v0 + vMax))*Abs(jMax))/(12.*Power(jMax,3)*vMax);
        profile.t[4] = Sqrt(-vf + vMax)/Sqrt(jMax);
        profile.t[5] = 0;
        profile.t[6] = Sqrt(-vf + vMax)/Sqrt(jMax);

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case6(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        double h1 = 5*Power(a0,2) + 6*a0*aMax + Power(aMax,2) + 2*jMax*v0;
        double h2 = 2*a0 + aMax;
        double h3 = 3*Power(a0,4) + 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) + 2*jMax*v0) + 12*jMax*(2*aMax*jMax*(p0 - pf) + Power(aMax,2)*(v0 + vf) + jMax*(Power(v0,2) - Power(vf,2)));
        double h4 = (a0 + aMax)*(Power(a0,2) + a0*aMax + 2*jMax*v0);
        double h5 = 4*Power(a0,4) + 8*Power(a0,3)*aMax + Power(aMax,4) + 24*aMax*Power(jMax,2)*(p0 - pf) - 24*a0*aMax*jMax*v0 + 4*Power(a0,2)*(Power(aMax,2) - 4*jMax*v0) + Power(aMax,2)*jMax*(-8*v0 + 12*vf) + 4*Power(jMax,2)*(4*Power(v0,2) - 3*Power(vf,2));
        double h6 = 1728*(2*Power(h1,3) - 6*h1*(h3 + 6*h2*h4) + 9*(Power(h2,2)*h3 + 12*Power(h4,2)))*Power(jMax,6);
        double h7 = Power(h6 + Sqrt(Power(h6,2) - 11943936*Power(h5,3)*Power(jMax,12)),0.3333333333333333);
        double h8 = Sqrt((4*Power(2,0.3333333333333333)*h5)/h7 + (Power(2,0.6666666666666666)*h7 + 24*(-2*h1 + 3*Power(h2,2))*Power(jMax,2))/(72.*Power(jMax,4)));

        profile.t[0] = -h2/(2.*jMax) + (-12*h8 + Sqrt((-576*Power(2,0.3333333333333333)*h5)/h7 - (2*Power(2,0.6666666666666666)*h7)/Power(jMax,4) - (96*(h1*(3*h2 + 2*h8*jMax) - 3*(Power(h2,3) + 2*h4 + Power(h2,2)*h8*jMax)))/(h8*Power(jMax,3))))/24.;
        profile.t[1] = 0;
        profile.t[2] = -aMax/(2.*jMax) + (-12*h8 + Sqrt((-576*Power(2,0.3333333333333333)*h5)/h7 - (2*Power(2,0.6666666666666666)*h7)/Power(jMax,4) - (96*(h1*(3*h2 + 2*h8*jMax) - 3*(Power(h2,3) + 2*h4 + Power(h2,2)*h8*jMax)))/(h8*Power(jMax,3))))/24.;
        profile.t[3] = 0;
        profile.t[4] = aMax/jMax;
        profile.t[5] = -(12*Power(a0,2)*aMax + jMax*(12*Power(aMax,2)*h8 + aMax*(-12*Power(h8,2)*jMax + h8*jMax*Sqrt((-576*Power(2,0.3333333333333333)*h5)/h7 - (2*Power(2,0.6666666666666666)*h7)/Power(jMax,4) - (96*(h1*(3*h2 + 2*h8*jMax) - 3*(Power(h2,3) + 2*h4 + Power(h2,2)*h8*jMax)))/(h8*Power(jMax,3))) - 24*v0) + h8*jMax*(h8*jMax*Sqrt((-576*Power(2,0.3333333333333333)*h5)/h7 - (2*Power(2,0.6666666666666666)*h7)/Power(jMax,4) - (96*(h1*(3*h2 + 2*h8*jMax) - 3*(Power(h2,3) + 2*h4 + Power(h2,2)*h8*jMax)))/(h8*Power(jMax,3))) + 24*vf)))/(24.*aMax*h8*Power(jMax,2));
        profile.t[6] = aMax/jMax;

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case7(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        double h1 = Power(aMax,2) + 2*jMax*vf;
        double h2 = 3*Power(a0,4) - 8*Power(a0,3)*aMax + 24*a0*aMax*jMax*v0 + 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) - 12*jMax*(2*aMax*jMax*(p0 - pf) + Power(aMax,2)*(v0 + vf) + jMax*(-Power(v0,2) + Power(vf,2)));
        double h3 = Power(jMax,4)*(-3*Power(a0,4) + 8*Power(a0,3)*aMax + Power(aMax,4) + 24*aMax*Power(jMax,2)*(p0 - pf) - 24*a0*aMax*jMax*v0 - 6*Power(a0,2)*(Power(aMax,2) - 2*jMax*v0) + 4*Power(aMax,2)*jMax*(3*v0 - 2*vf) + 4*Power(jMax,2)*(-3*Power(v0,2) + 4*Power(vf,2)));
        double h4 = 1728*Power(jMax,6)*(-2*Power(h1,3) - 6*h1*(h2 - 12*Power(aMax,2)*jMax*vf) + 9*Power(aMax,2)*(h2 - 48*Power(jMax,2)*Power(vf,2)));
        double h5 = Power(h4 + Sqrt(-11943936*Power(h3,3) + Power(h4,2)),0.3333333333333333);
        double h6 = Sqrt((-4*Power(2,0.3333333333333333)*h3)/(h5*Power(jMax,4)) - h5/(36.*Power(2,0.3333333333333333)*Power(jMax,4)) + Power(aMax,2)/Power(jMax,2) - (2*h1)/(3.*Power(jMax,2)));
        double h7 = Sqrt((288*Power(2,0.3333333333333333)*h3*h6 + h5*(Power(2,0.6666666666666666)*h5*h6 + 48*jMax*(3*Power(aMax,3) - 3*aMax*h1 + 3*Power(aMax,2)*h6*jMax - 2*h1*h6*jMax + 12*aMax*jMax*vf)))/(h5*h6*Power(jMax,4)))/(6.*Sqrt(2));

        profile.t[0] = (-a0 + aMax)/jMax;
        profile.t[1] = -(-Power(a0,2) + Power(aMax,2) + jMax*(h6*h7*jMax + 2*v0) + aMax*(-(h6*jMax) + h7*jMax - (2*vf)/h6))/(2.*aMax*jMax);
        profile.t[2] = aMax/jMax;
        profile.t[3] = 0;
        profile.t[4] = -(aMax + h6*jMax - h7*jMax)/(2.*jMax);
        profile.t[5] = 0;
        profile.t[6] = -(aMax + h6*jMax - h7*jMax)/(2.*jMax);

        profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
    }

    void time_UDDU_case8(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        if (std::abs(v0) < 1e-16 && std::abs(a0) < 1e-16 && std::abs(vf) < 1e-16) {
            profile.t[0] = Power((pf - p0) / (2 * jMax),1./3);
            profile.t[1] = 0;
            profile.t[2] = Power((pf - p0) / (2 * jMax),1./3);
            profile.t[3] = 0;
            profile.t[4] = Power((pf - p0) / (2 * jMax),1./3);
            profile.t[5] = 0;
            profile.t[6] = Power((pf - p0) / (2 * jMax),1./3);

            profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
            return;
        }

        double h1 = 2*Power(a0,3) + 3*Power(jMax,2)*(-p0 + pf) - 3*a0*jMax*(v0 - 2*vf);
        double h2 = Power(a0,2) + 2*jMax*(-v0 + vf);
        double h3 = Power(a0,5) - 24*Power(a0,2)*Power(jMax,2)*(p0 - pf) + 24*Power(jMax,3)*(-p0 + pf)*v0 + 4*Power(a0,3)*jMax*(v0 + 3*vf) + 12*a0*Power(jMax,2)*(Power(v0,2) + 2*v0*vf - Power(vf,2));
        double h4 = 3*Power(a0,4) - 24*a0*Power(jMax,2)*(p0 - pf) - 4*Power(jMax,2)*Power(v0 - vf,2) + 4*Power(a0,2)*jMax*(v0 + 5*vf);
        double h5 = Power(a0,6) - 48*Power(a0,3)*Power(jMax,2)*(p0 - pf) - 144*a0*Power(jMax,3)*(p0 - pf)*v0 + 6*Power(a0,4)*jMax*(v0 + 3*vf) + 36*Power(a0,2)*Power(jMax,2)*(Power(v0,2) + 2*v0*vf - Power(vf,2)) - 72*Power(jMax,3)*(jMax*Power(p0 - pf,2) - (v0 - vf)*Power(v0 + vf,2));
        double h6 = -(Power(jMax,4)*(Power(a0,8) - 192*Power(a0,5)*Power(jMax,2)*(p0 - pf) - 8*Power(a0,6)*jMax*(v0 - 5*vf) - 1152*a0*Power(jMax,4)*(p0 - pf)*v0*(v0 + vf) + 192*Power(a0,3)*Power(jMax,3)*(p0 - pf)*(5*v0 + 2*vf) + 120*Power(a0,4)*Power(jMax,2)*(Power(v0,2) - 2*v0*vf - 3*Power(vf,2)) - 96*Power(a0,2)*Power(jMax,3)*(3*jMax*Power(p0 - pf,2) + 5*Power(v0,3) - 3*Power(v0,2)*vf - 15*v0*Power(vf,2) + Power(vf,3)) + 48*Power(jMax,4)*(12*jMax*Power(p0 - pf,2)*(v0 + vf) + Power(v0 - vf,2)*(11*Power(v0,2) + 26*v0*vf + 11*Power(vf,2)))));
        double h7 = 3888*(36*h2*Power(h3,2) - 24*h1*h3*h4 + 3*Power(h4,3) + 16*Power(h1,2)*h5 - 12*h2*h4*h5);
        double h8 = (4*Power(h1,2) - 3*h2*h4)/(9.*Power(h2,2)*Power(jMax,2));
        double h9 = (-2*(8*Power(h1,3) + 9*Power(h2,2)*h3 - 9*h1*h2*h4))/(27.*Power(h2,3)*Power(jMax,3));
        double h10 = h7*Power(jMax,6) + Sqrt(-5038848*Power(h6,3) + Power(h7,2)*Power(jMax,12));
        auto h10_root = std::pow(std::complex<double>(h10, 0),0.3333333333333333);
        auto h11 = (Power(2,0.6666666666666666)*h10_root + (216*Power(2,0.3333333333333333)*h6)/h10_root)/(216.*h2*Power(jMax,4));
        auto h12 = std::sqrt(h11 + h8);
        auto h13_c = h12/2. - std::sqrt(-h11 + 2*h8 + h9/h12)/2. - h1/(3.*h2*jMax);
        auto h14_c = (h12 + std::sqrt(-h11 + 2*h8 + h9/h12))/2. - h1/(3.*h2*jMax);

        // Solution 4
        if (h14_c.real() > 0.0 && std::abs(h14_c.imag()) < 1e-10) {
            double h14 = h14_c.real();

            profile.t[0] = h14;
            profile.t[1] = 0;
            profile.t[2] = (-4*Power(a0,3) + 3*jMax*(h12*h2 + h2*std::sqrt(-h11 + 2*h8 + h9/h12) + 2*jMax*p0 - 2*jMax*pf) + 6*a0*(h2 + jMax*(v0 - 2*vf))).real()/(6.*h2*jMax);
            profile.t[3] = 0;
            profile.t[4] = (Power(a0,7) + 13*Power(a0,6)*h14*jMax + 72*Power(jMax,4)*(-(h14*(jMax*Power(p0 - pf,2) - Power(v0 - vf,3))) + Power(h14,2)*jMax*(p0 - pf)*(v0 - vf) + 2*(p0 - pf)*v0*(v0 - vf) + Power(h14,3)*jMax*Power(v0 - vf,2)) + 6*Power(a0,5)*jMax*(7*Power(h14,2)*jMax + v0 + 3*vf) - 12*Power(a0,3)*Power(jMax,2)*(10*h14*jMax*(p0 - pf) - Power(v0,2) + Power(h14,2)*jMax*(13*v0 - 16*vf) - 2*v0*vf + 3*Power(vf,2)) + 6*Power(a0,4)*Power(jMax,2)*(3*Power(h14,3)*jMax - 8*p0 + 8*pf + h14*(v0 + 19*vf)) - 36*Power(a0,2)*Power(jMax,3)*(Power(h14,2)*jMax*(p0 - pf) + 2*(-p0 + pf)*v0 + 2*Power(h14,3)*jMax*(v0 - vf) + h14*(3*Power(v0,2) + 2*v0*vf - 3*Power(vf,2))) - 72*a0*Power(jMax,3)*(Power(v0,3) + Power(v0,2)*vf - 3*v0*Power(vf,2) + Power(vf,3) + jMax*(Power(p0,2) + Power(pf,2) + h14*pf*(4*v0 - 2*vf) - 2*p0*(pf + 2*h14*v0 - h14*vf) + Power(h14,2)*(-2*Power(v0,2) + 5*v0*vf - 3*Power(vf,2)))))/(h5*jMax);
            profile.t[5] = 0;
            profile.t[6] = profile.t[4];

            if (profile.t[0] > 0.0 && profile.t[2] > 0.0 && profile.t[4] > 0.0) {
                profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
                return;
            }
        }

        // Solution 3
        if (h13_c.real() > 0.0 && std::abs(h13_c.imag()) < 1e-10) {
            double h13 = h13_c.real();

            profile.t[0] = h13;
            profile.t[1] = 0;
            profile.t[2] = (-4*Power(a0,3) + 3*jMax*(h12*h2 - h2*std::sqrt(-h11 + 2*h8 + h9/h12) + 2*jMax*p0 - 2*jMax*pf) + 6*a0*(h2 + jMax*(v0 - 2*vf))).real()/(6.*h2*jMax);
            profile.t[3] = 0;
            profile.t[4] = (Power(a0,7) + 13*Power(a0,6)*h13*jMax + 72*Power(jMax,4)*(-(h13*(jMax*Power(p0 - pf,2) - Power(v0 - vf,3))) + Power(h13,2)*jMax*(p0 - pf)*(v0 - vf) + 2*(p0 - pf)*v0*(v0 - vf) + Power(h13,3)*jMax*Power(v0 - vf,2)) + 6*Power(a0,5)*jMax*(7*Power(h13,2)*jMax + v0 + 3*vf) - 12*Power(a0,3)*Power(jMax,2)*(10*h13*jMax*(p0 - pf) - Power(v0,2) + Power(h13,2)*jMax*(13*v0 - 16*vf) - 2*v0*vf + 3*Power(vf,2)) + 6*Power(a0,4)*Power(jMax,2)*(3*Power(h13,3)*jMax - 8*p0 + 8*pf + h13*(v0 + 19*vf)) - 36*Power(a0,2)*Power(jMax,3)*(Power(h13,2)*jMax*(p0 - pf) + 2*(-p0 + pf)*v0 + 2*Power(h13,3)*jMax*(v0 - vf) + h13*(3*Power(v0,2) + 2*v0*vf - 3*Power(vf,2))) - 72*a0*Power(jMax,3)*(Power(v0,3) + Power(v0,2)*vf - 3*v0*Power(vf,2) + Power(vf,3) + jMax*(Power(p0,2) + Power(pf,2) + h13*pf*(4*v0 - 2*vf) - 2*p0*(pf + 2*h13*v0 - h13*vf) + Power(h13,2)*(-2*Power(v0,2) + 5*v0*vf - 3*Power(vf,2)))))/(h5*jMax);
            profile.t[5] = 0;
            profile.t[6] = profile.t[4];

            if (profile.t[0] > 0.0 && profile.t[2] > 0.0 && profile.t[4] > 0.0) {
                profile.set(p0, v0, a0, {jMax, 0, -jMax, 0, -jMax, 0, jMax});
                return;
            }
        }
    }

    void time_DUUD_case1(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case1(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case2(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case2(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case3(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case3(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case4(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case4(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case5(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case5(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case6(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case6(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case7(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case7(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    void time_DUUD_case8(Profile& profile, double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        time_UDDU_case8(profile, p0, v0, a0, pf, vf, -vMax, -aMax, -jMax);
        profile.set(p0, v0, a0, {-jMax, 0, jMax, 0, jMax, 0, -jMax});
    }

    double jerk_to_reach_target_with_times(const std::array<double, 7>& t, double p0, double v0, double a0, double pf) const {
        const double t1 {t[0]}, t2 {t[1]}, t3 {t[2]}, t4 {t[3]}, t5 {t[4]}, t6 {t[5]}, t7 {t[6]};
        return -((-6*p0 + 6*pf - 3*(t1 + t2 + t3 + t4 + t5 + t6 + t7)*(a0*(t1 + t2 + t3 + t4 + t5 + t6 + t7) + 2*v0))/(-Power(t1,3) + Power(t3,3) + Power(t5,3) + 3*Power(t5,2)*t6 + 3*t5*Power(t6,2) + 3*Power(t5,2)*t7 + 6*t5*t6*t7 + 3*t5*Power(t7,2) - Power(t7,3) + 3*Power(t3,2)*(t4 + t5 + t6 + t7) + 3*t3*Power(t4 + t5 + t6 + t7,2) - 3*Power(t1,2)*(t2 + t3 + t4 + t5 + t6 + t7) - 3*t1*Power(t2 + t3 + t4 + t5 + t6 + t7,2)));
    }

    Profile get_profile(double p0, double v0, double a0, double pf, double vf, double vMax, double aMax, double jMax) {
        Profile profile;

        // Test all cases to get ones that match

        time_UDDU_case1(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t > 0; })) {
            profile.type = ProfileType::UP_ACC0_ACC1_VEL;
            std::cout << "UP_ACC0_ACC1_VEL" << std::endl;
            return profile;
        }

        time_DUUD_case1(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t > 0; })) {
            profile.type = ProfileType::DOWN_ACC0_ACC1_VEL;
            std::cout << "DOWN_ACC0_ACC1_VEL" << std::endl;
            return profile;
        }

        time_UDDU_case3(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_ACC0;
            std::cout << "UP_ACC0" << std::endl;
            return profile;
        }

        time_DUUD_case3(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_ACC0;
            std::cout << "DOWN_ACC0" << std::endl;
            return profile;
        }

        time_UDDU_case4(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_ACC1;
            std::cout << "UP_ACC1" << std::endl;
            return profile;
        }

        time_DUUD_case4(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_ACC1;
            std::cout << "DOWN_ACC1" << std::endl;
            return profile;
        }

        time_UDDU_case5(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_ACC0_ACC1;
            std::cout << "UP_ACC0_ACC1" << std::endl;
            return profile;
        }

        time_DUUD_case5(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_ACC0_ACC1;
            std::cout << "DOWN_ACC0_ACC1" << std::endl;
            return profile;
        }

        time_UDDU_case2(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_VEL;
            std::cout << "UP_VEL" << std::endl;
            return profile;
        }

        time_DUUD_case2(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_VEL;
            std::cout << "DOWN_VEL" << std::endl;
            return profile;
        }

        time_UDDU_case6(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_ACC0_VEL;
            std::cout << "UP_ACC0_VEL" << std::endl;
            return profile;
        }

        time_DUUD_case6(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_ACC0_VEL;
            std::cout << "DOWN_ACC0_VEL" << std::endl;
            return profile;
        }

        time_UDDU_case7(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_ACC1_VEL;
            std::cout << "UP_ACC1_VEL" << std::endl;
            return profile;
        }

        time_DUUD_case7(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_ACC1_VEL;
            std::cout << "DOWN_ACC1_VEL" << std::endl;
            return profile;
        }

        time_UDDU_case8(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        // std::cout << "UP_NONE " << profile.t[0] << " " << profile.t[1] << " " << profile.t[2] << " " << profile.t[3] << " " << profile.t[4] << " " << profile.t[5] << " " << profile.t[6] << std::endl;
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::UP_NONE;
            std::cout << "UP_NONE" << std::endl;
            return profile;
        }

        time_DUUD_case8(profile, p0, v0, a0, pf, vf, vMax, aMax, jMax);
        // std::cout << "DOWN_NONE " << profile.t[0] << " " << profile.t[1] << " " << profile.t[2] << " " << profile.t[3] << " " << profile.t[4] << " " << profile.t[5] << " " << profile.t[6] << std::endl;
        if (std::all_of(profile.t.begin(), profile.t.end(), [](double t){ return t >= 0; })) {
            profile.type = ProfileType::DOWN_NONE;
            std::cout << "DOWN_NONE" << std::endl;
            return profile;
        }

        throw std::runtime_error("Could not find trajectory.");
    }

    bool calculate(const InputParameter<DOFs>& input) {
        current_input = input;

        // Check input
        if ((input.max_velocity.array() <= 0.0).any() || (input.max_acceleration.array() <= 0.0).any() || (input.max_jerk.array() <= 0.0).any()) {
            return false;
        }

        if ((input.target_velocity.array() != 0.0).any() || (input.target_acceleration.array() != 0.0).any()) {
            std::cerr << "Ruckig does not support a target velocity or acceleration." << std::endl;
            return false;
        }

        if (input.minimum_duration.has_value()) {
            std::cerr << "Ruckig does not support a minimum duration." << std::endl;
            return false;
        }

        auto start = std::chrono::high_resolution_clock::now();

        std::array<double, DOFs> tfs;
        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof]) {
                tfs[dof] = 0.0;
                continue;
            }

            profiles[dof] = get_profile(input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], input.target_position[dof], 0.0, input.max_velocity[dof], input.max_acceleration[dof], input.max_jerk[dof]);
            tfs[dof] = profiles[dof].t_sum[6];
        }

        auto tf_max_pointer = std::max_element(tfs.begin(), tfs.end());
        size_t limiting_dof = std::distance(tfs.begin(), tf_max_pointer);
        tf = *tf_max_pointer;

        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof] || dof == limiting_dof) {
                continue;
            }

            double new_jerk = jerk_to_reach_target_with_times(profiles[limiting_dof].t, input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], input.target_position[dof]);
            profiles[dof].t = profiles[limiting_dof].t;
            profiles[dof].reset(input.current_position[dof], input.current_velocity[dof], input.current_acceleration[dof], new_jerk);
        }

        auto stop = std::chrono::high_resolution_clock::now();
        last_calculation_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1000.0;

        t = 0.0;
        return true;
    }

    double t, tf;
    std::array<Profile, DOFs> profiles;

public:
    double delta_time;
    double last_calculation_duration {-1};

    explicit Ruckig(double delta_time): delta_time(delta_time) { }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        t += delta_time;

        if (input != current_input && !calculate(input)) {
            return Result::Error;
        }

        if (t + delta_time > tf) {
            output.new_position = input.target_position;
            output.new_velocity = input.target_velocity;
            output.new_acceleration = input.target_acceleration;
            return Result::Finished;
        }

        for (size_t dof = 0; dof < DOFs; dof += 1) {
            if (!input.enabled[dof]) {
                output.new_acceleration[dof] = input.current_acceleration[dof];
                output.new_velocity[dof] = input.current_velocity[dof];
                output.new_position[dof] = input.current_position[dof];
            }

            auto& p = profiles[dof];
            auto index_ptr = std::upper_bound(p.t_sum.begin(), p.t_sum.end(), t);
            size_t index = std::distance(p.t_sum.begin(), index_ptr);

            double t_diff = t;
            if (index > 0) {
                t_diff -= p.t_sum[index - 1];
            }

            output.new_acceleration[dof] = p.a[index] + t_diff * p.j[index];
            output.new_velocity[dof] = p.v[index] + t_diff * p.a[index] + 0.5 * std::pow(t_diff, 2) * p.j[index];
            output.new_position[dof] = p.p[index] + t_diff * p.v[index] + 0.5 * std::pow(t_diff, 2) * p.a[index] + 1. / 6 * std::pow(t_diff, 3) * p.j[index];
        }

        current_input.current_position = output.new_position;
        current_input.current_velocity = output.new_velocity;
        current_input.current_acceleration = output.new_acceleration;
        return Result::Working;
    }
};

} // namespace movex
