#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "frankx/frankx.hpp"
#include <affx/affine.hpp>

// this epsilon should be reflected, as Franka Emika is not perfect.
constexpr double epsilon = 0.001;

void testAffineApprox(affx::Affine a, affx::Affine b)
{
    REQUIRE(std::abs(a.x() - b.x()) < epsilon);
    REQUIRE(std::abs(a.y() - b.y()) < epsilon);
    REQUIRE(std::abs(a.z() - b.z()) < epsilon);
    REQUIRE(std::abs(a.qW() - b.qW()) < epsilon);
    REQUIRE(std::abs(a.qX() - b.qX()) < epsilon);
    REQUIRE(std::abs(a.qY() - b.qY()) < epsilon);
    REQUIRE(std::abs(a.qZ() - b.qZ()) < epsilon);
}

TEST_CASE("Test ForwardKinematics")
{
    try {
        frankx::Robot robot(TEST_ROBOT_HOSTNAME);
        affx::Affine currentAffine = robot.currentPose();
        std::array<double, 7> currentJointPose = robot.currentJointPositions();
        affx::Affine calcCurrentAffine = robot.forwardKinematics(currentJointPose);
        testAffineApprox(currentAffine, calcCurrentAffine);
    }
    catch (...)
    {
        REQUIRE(false);
    }
}

TEST_CASE("Test JointMotion")
{
    std::array<double, 7> testJointPose {-0.01981415, -1.036409, -0.05556389, -2.023421, 0.01193091, 1.796796, 1.770148};
    try {
        frankx::Robot robot(TEST_ROBOT_HOSTNAME, 0.5);
        movex::JointMotion targetJointMotion({testJointPose});
        robot.move(targetJointMotion);
        std::array<double, 7> current = robot.currentJointPositions();
        std::cout << robot.currentPose().toString() << std::endl;
        for(auto i = 0; i < 7; i++)
        {
            REQUIRE(std::abs(current.at(i) - testJointPose.at(i)) < epsilon);
        }
    }
    catch (...)
    {
        REQUIRE(false);
    }
}

std::stringstream AffineToString(affx::Affine a)
{
    std::stringstream  ss;
    ss << " x: " << a.x() << " y: " << a.y() << " z: " << a.z()
       << " q_w: " << a.qW() << " q_x: " << a.qX()
       << " q_y: " << a.qY() << " q_z: " << a.qZ();
    return ss;
}

TEST_CASE("Test WaypointMotion")
{
//    x: 0.181861 y: -0.0318449 z: 0.835031 q_w: 0.557629 q_x: 0.316947 q_y: -0.235395 q_z: -0.730194

    affx::Affine testAffine {0.182218, -0.032166, 0.834696, -1.752703, 0.200432, -2.349596};
    try {
        frankx::Robot robot(TEST_ROBOT_HOSTNAME, 0.5);
        movex::Waypoint waypoint{testAffine};
        movex::WaypointMotion targetWaypointMotion({waypoint});
        robot.move(targetWaypointMotion);
        affx::Affine currentAffine = robot.currentPose();
        std::cout << "testAffine: " << AffineToString(currentAffine).str() << std::endl;
        std::cout << "currentAffine: " << AffineToString(currentAffine).str() << std::endl;
        testAffineApprox(currentAffine, testAffine);
    }
    catch (...)
    {
        REQUIRE(false);
    }
}
