#include <iostream>

#include <frankx/kinematics.hpp>


namespace frankx {

std::array<double, 16> Kinematics::forward(const Eigen::Matrix<double, 7, 1>& q) {
    const double s0 = std::sin(q[0]), c0 = std::cos(q[0]);
    const double s1 = std::sin(q[1]), c1 = std::cos(q[1]);
    const double s2 = std::sin(q[2]), c2 = std::cos(q[2]);
    const double s3 = std::sin(q[3]), c3 = std::cos(q[3]);
    const double s4 = std::sin(q[4]), c4 = std::cos(q[4]);
    const double s5 = std::sin(q[5]), c5 = std::cos(q[5]);
    const double s6 = std::sin(q[6]), c6 = std::cos(q[6]);

    double c_p_s6 = c6 + s6;
    double c_m_s6 = c6 - s6;

    const double t1 = c3*(c5*c4*c_m_s6 + s4*c_p_s6) - s3*s5*c_m_s6;
    const double t3 = c3*(c5*c4*c_p_s6 - s4*c_m_s6) - s3*s5*c_p_s6;
    const double t2  = c4*c_p_s6 - c5*s4*c_m_s6;
    const double t18 = c4*c_m_s6 + c5*s4*c_p_s6;
    const double t20 = c3*s5*c_p_s6 - s3*s4*c_m_s6;
    const double t21 = c3*s5*c_m_s6 + s3*s4*c_p_s6;
    const double t4 = s1*(t20 + c5*c4*s3*c_p_s6) + c1*(c2*t3 - s2*t18);
    const double t5 = s1*(t21 + c5*c4*s3*c_m_s6) + c1*(c2*t1 + s2*t2);
    const double t8 = -0.088*c5 - 0.107*s5;
    const double t22 = 0.384 + 0.088*s5 - 0.107*c5;
    const double t6 = -0.0825 - c4*t8;
    const double t7 = 0.316 + c3*t22 + s3*t6;
    const double t9 = -0.0825 + s3*t22 - c3*t6;
    const double t13 = c1*t21 + c5*s4*s1*s2*c_m_s6;
    const double t14 = c1*t20 + c5*s4*s1*s2*c_p_s6;
    const double t15 = c2*t18 + s2*t3;
    const double t16 = c2*t2 - s2*t1;
    const double t17 = s1*s3 + c1*c2*c3;
    const double t24 = s1*c3 - c1*c2*s3;
    const double t19 = c1*(-c2*t9 + s2*s4*t8) + s1*t7;
    const double t23 = s2*t9 + c2*s4*t8;

    const double sq2 = 1./std::sqrt(2);

    return {
        (s0*t16 + c0*t5)*sq2,
        (-c0*t16 + s0*t5)*sq2,
        (t13 - c4*(s1*s2*c_p_s6 - c1*c5*s3*c_m_s6) - c2*s1*t1)*sq2,
        0,
        (-s0*t15 + c0*t4)*sq2,
        (c0*t15 + s0*t4)*sq2,
        (t14 + c4*(s1*s2*c_m_s6 + c1*c5*s3*c_p_s6) - c2*s1*t3)*sq2,
        0,
        c5*(c0*t24 + s3*s0*s2) + c4*s5*(c3*s0*s2 - c0*t17) + (c2*s0 + c0*c1*s2)*s4*s5,
        c5*(s0*t24 - s3*c0*s2) - c4*s5*(c3*c0*s2 + s0*t17) - (c2*c0 - s0*c1*s2)*s4*s5,
        c1*(c3*c5 - s3*c4*s5) + s1*(c2*(c5*s3 + c3*c4*s5) - s2*s4*s5),
        0,
        s0*t23 + c0*t19,
        s0*t19 - c0*t23,
        0.333 - s1*s2*s4*t8 + c2*s1*t9 + c1*t7,
        1,
    };
}


Eigen::Matrix<double, 6, 1> Kinematics::forwardEuler(const Eigen::Matrix<double, 7, 1>& q) {
    const double s0 = std::sin(q[0]), c0 = std::cos(q[0]);
    const double s1 = std::sin(q[1]), c1 = std::cos(q[1]);
    const double s2 = std::sin(q[2]), c2 = std::cos(q[2]);
    const double s3 = std::sin(q[3]), c3 = std::cos(q[3]);
    const double s4 = std::sin(q[4]), c4 = std::cos(q[4]);
    const double s5 = std::sin(q[5]), c5 = std::cos(q[5]);
    const double s6 = std::sin(q[6]), c6 = std::cos(q[6]);

    double c_p_s6 = c6 + s6;
    double c_m_s6 = c6 - s6;

    const double t1 = c3*(c5*c4*c_m_s6 + s4*c_p_s6) - s3*s5*c_m_s6;
    const double t3 = c3*(c5*c4*c_p_s6 - s4*c_m_s6) - s3*s5*c_p_s6;
    const double t2  = c4*c_p_s6 - c5*s4*c_m_s6;
    const double t18 = c4*c_m_s6 + c5*s4*c_p_s6;
    const double t20 = c3*s5*c_p_s6 - s3*s4*c_m_s6;
    const double t21 = c3*s5*c_m_s6 + s3*s4*c_p_s6;
    const double t4 = s1*(t20 + c5*c4*s3*c_p_s6) + c1*(c2*t3 - s2*t18);
    const double t5 = s1*(t21 + c5*c4*s3*c_m_s6) + c1*(c2*t1 + s2*t2);
    const double t8 = -0.088*c5 - 0.107*s5;
    const double t22 = 0.384 + 0.088*s5 - 0.107*c5;
    const double t6 = -0.0825 - c4*t8;
    const double t7 = 0.316 + c3*t22 + s3*t6;
    const double t9 = -0.0825 + s3*t22 - c3*t6;
    const double t14 = c1*t20 + c5*s4*s1*s2*c_p_s6;
    const double t15 = c2*t18 + s2*t3;
    const double t16 = c2*t2 - s2*t1;
    const double t17 = s1*s3 + c1*c2*c3;
    const double t24 = s1*c3 - c1*c2*s3;
    const double t19 = c1*(-c2*t9 + s2*s4*t8) + s1*t7;
    const double t23 = s2*t9 + c2*s4*t8;

    const double sq2 = 1./std::sqrt(2);

    const double a21 = (-c0*t16 + s0*t5);
    const double a22 = (c0*t15 + s0*t4);
    const double a31 = c5*(c0*t24 + s3*s0*s2) + c4*s5*(c3*s0*s2 - c0*t17) + (c2*s0 + c0*c1*s2)*s4*s5;
    const double a32 = (t14 + c4*(s1*s2*c_m_s6 + c1*c5*s3*c_p_s6) - c2*s1*t3)*sq2;
    const double a33 = c1*(c3*c5 - s3*c4*s5) + s1*(c2*(c5*s3 + c3*c4*s5) - s2*s4*s5);

    const double e1 = std::atan(a21/a22);
    const double e2 = std::asin(a31);
    const double e3 = std::atan(a32/a33);

    return (Eigen::Matrix<double, 6, 1>() <<
        c0*t19 + s0*t23,
        s0*t19 - c0*t23,
        0.333 - s1*s2*s4*t8 + c2*s1*t9 + c1*t7,
        e1,
        e2,
        e3
    ).finished();
}


Eigen::Matrix<double, 6, 7> Kinematics::jacobian(const Eigen::Matrix<double, 7, 1>& q) {
    const double s0 = std::sin(q[0]), c0 = std::cos(q[0]);
    const double s1 = std::sin(q[1]), c1 = std::cos(q[1]);
    const double s2 = std::sin(q[2]), c2 = std::cos(q[2]);
    const double s3 = std::sin(q[3]), c3 = std::cos(q[3]);
    const double s4 = std::sin(q[4]), c4 = std::cos(q[4]);
    const double s5 = std::sin(q[5]), c5 = std::cos(q[5]);
    const double s6 = std::sin(q[6]), c6 = std::cos(q[6]);

    const double p0 = 0.088*c5 + 0.107*s5;
    const double p1 = -0.384 + 0.107*c5 - 0.088*s5;
    const double p2 = c4*c5*(c6 - s6) + s4*(c6 + s6);
    const double p3 = s4*c5*(s6 - c6) + c4*(c6 + s6);
    const double p4 = s3*s5*(-c6 + s6) + c3*p2;
    const double p5 = -0.0825 + c4*p0;
    const double p6 = 0.0825 + c3*p5 + s3*p1;

    const double t5 = (c0*(-(c2*p3) + s2*p4) + s0*(s1*(c6*s3*s4 + c3*c6*s5 + c4*c5*s3*(c6 - s6) + s3*s4*s6 - c3*s5*s6) + c1*(s2*p3 + c2*p4)));
    const double t6 = (s0*(c2*p3 - s2*p4) + c0*(s1*(c6*s3*s4 + c3*c6*s5 + c4*c5*s3*(c6 - s6) + s3*s4*s6 - c3*s5*s6) + c1*(s2*p3 + c2*p4)));
    const double t7 = (c5*(c6 + s6)*s1*s2*s4 + c1*c3*(s6 + c6)*s5 + c1*s3*s4*(s6 - c6) + c4*(c6*s1*s2 + c1*c5*c6*s3 - s1*s2*s6 + c1*c5*s3*s6) + c2*s1*(s3*s5*(c6 + s6) - c3*(s4*(-c6 + s6) + c4*c5*(c6 + s6))));
    const double t8 = (s1*(c6*s3*s4 - c3*c6*s5 + c4*c5*s3*(-c6 - s6) - s3*s4*s6 - c3*s5*s6) + c1*(s2*(c4*(c6 - s6) + c5*s4*(c6 + s6)) + c2*(c3*(c4*c5*(-c6 - s6) + s4*(c6 - s6)) + s3*s5*(c6 + s6))));
    const double t9 = (s1*(c4*c6*s3 - c5*s3*s4*(c6 - s6) + c4*s3*s6) + c1*(c2*c3*(-(c5*s4*(c6 - s6)) + c4*(c6 + s6)) + s2*(c4*c5*(-c6 + s6) - s4*(c6 + s6))));

    const double t1 = t7*t7;
    const double t2 = t6*t6;
    const double t3 = t5*t5;
    const double t4 = std::sqrt(1 - std::pow(c5*(c6 - s6)*s1*s2*s4 + c1*(c6 + s6)*s3*s4 + c1*c3*(c6 - s6)*s5 - c4*((c6 + s6)*s1*s2 - c1*c5*c6*s3 + c1*c5*s3*s6) - c2*s1*p4,2)/2.);


    Eigen::Matrix<double, 6, 7> result;
    result(0, 0) = c0*(s2*(-0.0825 + c3*(-p5) + s3*(-p1)) - c2*s4*p0) - s0*(c1*(c2*p6 - s2*s4*p0) + s1*(0.316 - c3*p1 + s3*p5));
    result(1, 0) = c0*c1*(c2*p6 - s2*s4*p0) - s0*(s2*p6 + c2*s4*p0) + c0*s1*(0.316 - c3*p1 + s3*p5);
    result(2, 0) = 0;
    result(3, 0) = 1;
    result(4, 0) = 0;
    result(5, 0) = 0;
    result(0, 1) = c0*(-(s1*(c2*p6 - s2*s4*p0)) + c1*(0.316 - c3*p1 + s3*p5));
    result(1, 1) = s0*(-(s1*(c2*p6 - s2*s4*p0)) + c1*(0.316 - c3*p1 + s3*p5));
    result(2, 1) = s1*(-0.316 + 0.0825*s3 - 0.088*c4*c5*s3 + c3*p1 - 0.107*c4*s3*s5) + c1*(s2*s4*p0 + c2*(-0.0825 + 0.384*s3 - 0.107*c5*s3 + 0.088*s3*s5 + c3*(0.0825 - 0.088*c4*c5 - 0.107*c4*s5)));
    result(3, 1) = ((s0*(c1*(c6*s3*s4 + c3*c6*s5 + c4*c5*s3*(c6 - s6) + s3*s4*s6 - c3*s5*s6) - s1*(s2*p3 + c2*p4)))/t6 - (c0*(c1*(c6*s3*s4 + c3*c6*s5 + c4*c5*s3*(c6 - s6) + s3*s4*s6 - c3*s5*s6) - s1*(s2*p3 + c2*p4))*t5)/t2)/(1 + t3/t2);
    result(4, 1) = -((c1*c5*c6*s2*s4 - c6*s1*s3*s4 - c3*c6*s1*s5 - c1*c5*s2*s4*s6 - s1*s3*s4*s6 + c3*s1*s5*s6 - c4*(c1*c6*s2 + c5*c6*s1*s3 + c1*s2*s6 - c5*s1*s3*s6) - c1*c2*p4)/(std::sqrt(2)*t4));
    result(5, 1) = ((c1*c5*c6*s2*s4 + c6*s1*s3*s4 - c3*c6*s1*s5 + c1*c5*s2*s4*s6 - s1*s3*s4*s6 - c3*s1*s5*s6 + c4*(c1*c6*s2 - c5*c6*s1*s3 - c1*s2*s6 - c5*s1*s3*s6) + c1*c2*(s3*s5*(c6 + s6) - c3*(s4*(-c6 + s6) + c4*c5*(c6 + s6))))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))) - ((-(s1*(c3*c5 - c4*s3*s5)) + c1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))*t7)/(std::sqrt(2)*t2))/(1 + t1/(2.*t2));
    result(0, 2) = c0*c1*(-(s2*p6) - c2*s4*p0) + s0*(c2*(-0.0825 + c3*(-p5) - s3*p1) + s2*s4*p0);
    result(1, 2) = c1*s0*(-(s2*p6) - c2*s4*p0) + c0*(c2*p6 - s2*s4*p0);
    result(2, 2) = 0.088*c2*c5*s1*s4 - s1*s2*(-0.0825 + c3*(-p5) + s3*(-p1)) + 0.107*c2*s1*s4*s5;
    result(3, 2) = ((c0*(s2*p3 + c2*p4) + c1*s0*(c2*p3 - s2*p4))/t6 - ((s0*(-(s2*p3) - c2*p4) + c0*c1*(c2*p3 - s2*p4))*t5)/t2)/(1 + t3/t2);
    result(4, 2) = -((c2*c5*c6*s1*s4 - c2*c5*s1*s4*s6 - c4*(c2*c6*s1 + c2*s1*s6) + s1*s2*p4)/(std::sqrt(2)*t4));
    result(5, 2) = (-((s1*(-(c2*s4*s5) - s2*(c5*s3 + c3*c4*s5))*t7)/(std::sqrt(2)*t2)) + (c2*c5*c6*s1*s4 + c2*c5*s1*s4*s6 + c4*(c2*c6*s1 - c2*s1*s6) - s1*s2*(s3*s5*(c6 + s6) - c3*(s4*(-c6 + s6) + c4*c5*(c6 + s6))))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))))/(1 + t1/(2.*t2));
    result(0, 3) = s0*s2*(s3*p5 - c3*p1) + c0*(c1*c2*(-(s3*p5) + c3*p1) + s1*(s3*p1 + c3*p5));
    result(1, 3) = c1*c2*s0*(-(s3*p5) + c3*p1) + c0*s2*(-(s3*p5) + c3*p1) + s0*s1*(s3*p1 + c3*p5);
    result(2, 3) = c2*s1*(s3*p5 - c3*p1) + c1*(-(s3*(-p1)) + c3*p5);
    result(3, 3) = ((c0*s2*(c3*s5*(-c6 + s6) - s3*p2) + s0*(s1*(c3*c6*s4 - c6*s3*s5 + c3*c4*c5*(c6 - s6) + c3*s4*s6 + s3*s5*s6) + c1*c2*(c3*s5*(-c6 + s6) - s3*p2)))/t6 - ((-(s0*s2*(c3*s5*(-c6 + s6) - s3*p2)) + c0*(s1*(c3*c6*s4 - c6*s3*s5 + c3*c4*c5*(c6 - s6) + c3*s4*s6 + s3*s5*s6) + c1*c2*(c3*s5*(-c6 + s6) - s3*p2)))*t5)/t2)/(1 + t3/t2);
    result(4, 3) = -((c1*c3*(c6 + s6)*s4 + c1*s3*s5*(s6 - c6) - c4*c1*c3*c5*(s6 - c6) - c2*s1*(c3*s5*(-c6 + s6) - s3*p2))/(std::sqrt(2)*t4));
    result(5, 3) = (-(((c1*(-(c5*s3) - c3*c4*s5) + c2*s1*(c3*c5 - c4*s3*s5))*t7)/(std::sqrt(2)*2)) + (-(c1*c3*c6*s4) - c1*c6*s3*s5 + c1*c3*s4*s6 - c1*s3*s5*s6 + c4*(c1*c3*c5*c6 + c1*c3*c5*s6) + c2*s1*(c3*s5*(c6 + s6) + s3*(s4*(-c6 + s6) + c4*c5*(c6 + s6))))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))))/(1 + t1/(2.*t2));
    result(0, 4) = s0*(c2*c4*(-p0) + c3*s2*s4*p0) + c0*(c1*(c4*s2*(-p0) - c2*c3*s4*p0) + s1*s3*(-0.088*c5*s4 - 0.107*s4*s5));
    result(1, 4) = c1*s0*(c4*s2*(-p0) - c2*c3*s4*p0) + c0*(c2*c4*p0 - c3*s2*s4*p0) + s0*s1*s3*(-0.088*c5*s4 - 0.107*s4*s5);
    result(2, 4) = 0.088*c4*c5*s1*s2 - c2*c3*s1*s4*(-p0) + 0.107*c4*s1*s2*s5 + c1*s3*(-0.088*c5*s4 - 0.107*s4*s5);
    result(3, 4) = ((c0*(c3*s2*(-(c5*s4*(c6 - s6)) + c4*(c6 + s6)) - c2*(c4*c5*(-c6 + s6) - s4*(c6 + s6))) + s0*t9)/t6 - ((s0*(-(c3*s2*(-(c5*s4*(c6 - s6)) + c4*(c6 + s6))) + c2*(c4*c5*(-c6 + s6) - s4*(c6 + s6))) + c0*t9)*t5)/t2)/(1 + t3/t2);
    result(4, 4) = -((c4*c5*c6*s1*s2 + c1*c4*c6*s3 - c4*c5*s1*s2*s6 + c1*c4*s3*s6 + s4*(c6*s1*s2 - c1*c5*c6*s3 + s1*s2*s6 + c1*c5*s3*s6) - c2*c3*s1*(-(c5*s4*(c6 - s6)) + c4*(c6 + s6)))/(std::sqrt(2)*t4));
    result(5, 4) = ((c4*c5*c6*s1*s2 - c1*c4*c6*s3 + c4*c5*s1*s2*s6 + c1*c4*s3*s6 - s4*(c6*s1*s2 + c1*c5*c6*s3 - s1*s2*s6 + c1*c5*s3*s6) - c2*c3*s1*(c4*(-c6 + s6) - c5*s4*(c6 + s6)))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))) - ((c1*s3*s4*s5 + s1*(-(c4*s2*s5) - c2*c3*s4*s5))*t7)/(std::sqrt(2)*t2))/(1 + t1/(2.*t2));
    result(0, 5) = s0*(s2*(c3*c4*(-0.107*c5 + 0.088*s5) + s3*p0) + c2*s4*(-0.107*c5 + 0.088*s5)) + c0*(c1*(c2*(-s3*p0 + c3*c4*(0.107*c5 - 0.088*s5)) + s2*s4*(-0.107*c5 + 0.088*s5)) + s1*(c3*p0 + s3*(0.107*c4*c5 - 0.088*c4*s5)));
    result(1, 5) = c0*(s2*(s3*(-p0) + c3*c4*(0.107*c5 - 0.088*s5)) + c2*s4*(0.107*c5 - 0.088*s5)) + c1*s0*(c2*(-s3*p0 + c3*c4*(0.107*c5 - 0.088*s5)) + s2*s4*(-0.107*c5 + 0.088*s5)) + s0*s1*(c3*p0 + s3*(0.107*c4*c5 - 0.088*c4*s5));
    result(2, 5) = 0.107*c5*s1*s2*s4 + c2*s1*(c3*c4*(-0.107*c5 + 0.088*s5) + s3*p0) - 0.088*s1*s2*s4*s5 + c1*(c3*p0 + s3*(0.107*c4*c5 - 0.088*c4*s5));
    result(3, 5) = ((c0*(c2*s4*s5*(-c6 + s6) + s2*(-(c3*c4*s5*(c6 - s6)) + c5*s3*(-c6 + s6))) + s0*(s1*(c3*c5*c6 - c4*s3*s5*(c6 - s6) - c3*c5*s6) + c1*(-(s2*s4*s5*(-c6 + s6)) + c2*(-(c3*c4*s5*(c6 - s6)) + c5*s3*(-c6 + s6)))))/t6 - ((s0*(-(c2*s4*s5*(-c6 + s6)) - s2*(-(c3*c4*s5*(c6 - s6)) + c5*s3*(-c6 + s6))) + c0*(s1*(c3*c5*c6 - c4*s3*s5*(c6 - s6) - c3*c5*s6) + c1*(-(s2*s4*s5*(-c6 + s6)) + c2*(-(c3*c4*s5*(c6 - s6)) + c5*s3*(-c6 + s6)))))*t5)/t2)/(1 + t3/t2);
    result(4, 5) = -((c1*c3*c5*(c6 - s6) - c6*s1*s2*s4*s5 + s1*s2*s4*s5*s6 - c4*c1*(c6 - s6)*s3*s5 - c2*s1*(-(c3*c4*s5*(c6 - s6)) + c5*s3*(-c6 + s6)))/(std::sqrt(2)*t4));
    result(5, 5) = ((c1*c3*c5*(c6 + s6) - c6*s1*s2*s4*s5 - s1*s2*s4*s5*s6 - c4*c1*(c6 + s6)*s3*s5 + c2*s1*(c5*s3*(c6 + s6) + c3*c4*s5*(c6 + s6)))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))) - ((c1*(-(c4*c5*s3) - c3*s5) + s1*(-(c5*s2*s4) + c2*(c3*c4*c5 - s3*s5)))*t7)/(std::sqrt(2)*t2))/(1 + t1/(2.*t2));
    result(0, 6) = 0;
    result(1, 6) = 0;
    result(2, 6) = 0;
    result(3, 6) = ((c0*(-(c2*(c4*(c6 - s6) + c5*s4*(c6 + s6))) + s2*(c3*(c4*c5*(-c6 - s6) + s4*(c6 - s6)) + s3*s5*(c6 + s6))) + s0*t8)/t6 - ((s0*(c2*(c4*(c6 - s6) + c5*s4*(c6 + s6)) - s2*(c3*(c4*c5*(-c6 - s6) + s4*(c6 - s6)) + s3*s5*(c6 + s6))) + c0*t8)*t5)/t2)/(1 + t3/t2);
    result(4, 6) = -((-(c5*c6*s1*s2*s4) + c1*c6*s3*s4 - c1*c3*c6*s5 - c5*s1*s2*s4*s6 - c1*s3*s4*s6 - c1*c3*s5*s6 - c4*(c6*s1*s2 + c1*c5*c6*s3 - s1*s2*s6 + c1*c5*s3*s6) - c2*s1*(c3*(c4*c5*(-c6 - s6) + s4*(c6 - s6)) + s3*s5*(c6 + s6)))/(std::sqrt(2)*t4));
    result(5, 6) = (c5*c6*s1*s2*s4 + c1*c6*s3*s4 + c1*c3*c6*s5 - c5*s1*s2*s4*s6 + c1*s3*s4*s6 - c1*c3*s5*s6 + c4*(-(c6*s1*s2) + c1*c5*c6*s3 - s1*s2*s6 - c1*c5*s3*s6) + c2*s1*(s3*s5*(c6 - s6) - c3*p2))/(std::sqrt(2)*(c1*(c3*c5 - c4*s3*s5) + s1*(-(s2*s4*s5) + c2*(c5*s3 + c3*c4*s5)))*(1 + t1/(2.*t2)));
    return result;
}


Eigen::Matrix<double, 7, 1> Kinematics::inverse(const Eigen::Matrix<double, 6, 1>& x_target, const Eigen::Matrix<double, 7, 1>& q0, std::optional<NullSpaceHandling> null_space) {
    const double tolerance {1e-10};
    auto eye = Eigen::Matrix<double, 7, 7>::Identity();

    Eigen::Matrix<double, 6, 1> x_current;
    Eigen::Matrix<double, 7, 1> q_current = q0;

    for (size_t i = 0; i < 80; ++i) {
        x_current = forwardEuler(q_current);

        auto j = jacobian(q_current);
        auto j_inv = Kinematics::pseudoinverse(j);

        // Null-space handling
        Eigen::Matrix<double, 7, 1> dq_0 = Eigen::Matrix<double, 7, 1>::Zero();
        if (null_space) {
            dq_0(null_space->joint_index) = 5.0 * (null_space->value - q_current(null_space->joint_index));
        }

        auto dq = j_inv * (x_target - x_current) + (eye - j_inv * j) * dq_0;

        // Line search
        double alpha_min {1.0};
        double dis_min {1000.0};
        for (size_t ii = 0; ii < 20; ++ii) {
            double alpha = 0.1 * ii;
            auto x_new = forwardEuler(q_current + alpha * dq);
            double new_dis = (x_target - x_new).squaredNorm();

            if (new_dis < dis_min) {
                dis_min = new_dis;
                alpha_min = alpha;
            }
        }
        q_current += alpha_min * dq;

        if (dis_min < tolerance) {
            break;
        }

        // std::cout << i << " dis_min: " << dis_min << " " << (q0 - q_current).squaredNorm() << std::endl;
    }
    return q_current;
}

}
