#include <iostream>

#include <frankx/frankx.hpp>

using namespace frankx;

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Connect to the robot
    Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot.setDynamicRel(0.5);

    std::array<double, 7> home = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    auto motion_home = JointMotion(home);
    robot.move(motion_home);

    return 0;
}