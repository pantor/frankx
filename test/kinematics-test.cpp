#define CATCH_CONFIG_MAIN
#include <iostream>
#include <chrono>

#include <Eigen/Core>

#include <frankx/frankx.hpp>
#include <catch2/catch.hpp>

using namespace frankx;

// vector of corresponding pairs Affine (XYZ+Quaternion) and JointPositions
std::vector<std::pair<std::array<double, 7>, std::array<double,7>>> test_poses
        {
                {
                        {0.01291572, -0.219187, 0.5566107, 0.01460959, 0.9677397, -0.2340099, -0.09222686},
                        {0.4401378, -1.720145, -0.4573823, -2.394063, -0.6454046, 0.5917417, 0.4929689}
                },
                {
                        {0.1170532, -0.06754467, 0.6470401, -0.406576, 0.7517108, -0.3960603, 0.3358022},
                        {0.3027361, -1.755932, -0.3099454, -2.759472, -0.1217491, 2.112061, 1.139796}
                },
                {
                        {0.1146934, -0.08584384, 0.5948072, 0.1094964, 0.8487899, -0.2910642, 0.4276072},
                        {-0.4999438, -1.700211, -0.06022732, -2.897249, -0.1245556, 2.109992, 0.1506971}
                }
        };


std::string affineToString(affx::Affine a)
{
    std::stringstream  ss;
    ss << std::setprecision(5);
    ss << " x: " << a.x() << " y: " << a.y() << " z: " << a.z()
       << " q_w: " << a.qW() << " q_x: " << a.qX()
       << " q_y: " << a.qY() << " q_z: " << a.qZ()
       << " a: " << a.a() << " b: " << a.b() << " c: " << a.c();
    return ss.str();
}

TEST_CASE("Test Kinematics forward")
{
    std::cout << "forward" << std::endl;
    for (auto pose: test_poses)
    {
        Affine correct = Affine(
                pose.first[0], pose.first[1],
                pose.first[2],
                pose.first[3], pose.first[4],
                pose.first[5], pose.first[6]);
        const Eigen::Matrix<double, 7, 1> q_current =
                Eigen::Map<const Eigen::Matrix<double, 7, 1>>(pose.second.data(), pose.second.size());
        Affine calc = Affine(frankx::Kinematics::forward(q_current));
        std::cout << "correct: " << affineToString(correct) << std::endl;
        std::cout << "calc:    " << affineToString(calc) << std::endl;
        std::cout << "angularDistance" << correct.quaternion().angularDistance(calc.quaternion()) << std::endl;
        REQUIRE(calc.isApprox(correct));
    }
}

TEST_CASE("Test Kinematics forwardEuler")
{
    std::cout << "forwardEuler" << std::endl;
    for (auto pose: test_poses)
    {
        Affine correct = Affine(
                pose.first[0], pose.first[1],
                pose.first[2],
                pose.first[3], pose.first[4],
                pose.first[5], pose.first[6]);
        const Eigen::Matrix<double, 7, 1> q_current =
                Eigen::Map<const Eigen::Matrix<double, 7, 1>>(pose.second.data(), pose.second.size());
        Affine calc = Affine(frankx::Kinematics::forwardEuler(q_current));
        std::cout << "correct: " << affineToString(correct) << std::endl;
        std::cout << "calc:    " << affineToString(calc) << std::endl;
        std::cout << "angularDistance" << correct.quaternion().angularDistance(calc.quaternion()) << std::endl;
        REQUIRE(calc.isApprox(correct));
    }
}

TEST_CASE("Test Kinematics inverse duration"){
    std::array<double, 7> q = {-1.4554923355, 1.1540154275, 1.50061583024, -2.30909621308, -1.3141626213, 1.93919787437, 0.028150367940};
    std::array<double, 6> x = {0.473971, -0.307686, 0.340767, 0.545131, -0.510650, -0.552355};

    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix<double, 6, 1> x_target = Eigen::Map<Eigen::Matrix<double, 6, 1>>(x.data(), x.size());
    Eigen::Matrix<double, 7, 1> q_current = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q.data(), q.size());

    auto q_new = Kinematics::inverse(x_target, q_current, Kinematics::NullSpaceHandling {1, 1.0});

    auto stop = std::chrono::high_resolution_clock::now();
    double t = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1000.0;

    std::cout << "q_new: " << q_new << std::endl;
    std::cout << "calculation time: " << t << std::endl;
}
