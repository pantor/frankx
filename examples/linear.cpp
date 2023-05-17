#include <franky/franky.hpp>
#include "util.cpp"


using namespace franky;


int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Connect to the robot
    Robot robot(argv[1]);
    robot.automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot.setDynamicRel(0.15);

    // Define and move forwards
    auto way = mk_affine(0.0, 0.05, 0.0);
    auto motion_forward = LinearRelativeMotion(way);
    robot.move(motion_forward);

    // And move backwards using the inverse motion
    auto motion_backward = LinearRelativeMotion(way.inverse());
    robot.move(motion_backward);

    return 0;
}