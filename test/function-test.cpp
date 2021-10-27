#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "frankx/frankx.hpp"
#include <affx/affine.hpp>

// this epsilon should be reflected, as Franka Emika is not perfect.
constexpr double epsilon = 0.001;

std::string AffineToString(affx::Affine a)
{
    std::stringstream  ss;
    ss << " x: " << a.x() << " y: " << a.y() << " z: " << a.z()
       << " q_w: " << a.qW() << " q_x: " << a.qX()
       << " q_y: " << a.qY() << " q_z: " << a.qZ();
    return ss.str();
}

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


TEST_CASE("Test WaypointMotion")
{
    affx::Affine testAffine {0.182016, -0.0320245, 0.834567, -0.236185, -0.382481, 0.0959011, 0.888104};
    try {
        frankx::Robot robot(TEST_ROBOT_HOSTNAME, 0.5);
        movex::Waypoint waypoint{testAffine};
        movex::WaypointMotion targetWaypointMotion({waypoint});
        robot.move(targetWaypointMotion);
        affx::Affine currentAffine = robot.currentPose();
        std::cout << "testAffine: " << AffineToString(currentAffine) << std::endl;
        std::cout << "currentAffine: " << AffineToString(currentAffine) << std::endl;
        testAffineApprox(currentAffine, testAffine);
    }
    catch (...)
    {
        REQUIRE(false);
    }
}
