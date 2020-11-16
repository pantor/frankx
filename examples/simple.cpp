#include <frankx/frankx.hpp>


using namespace frankx;


int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    // Connect to the robot
    Robot robot(argv[1]);
    robot.setDefaultBehavior();
    robot.automaticErrorRecovery();

    // Reduce the acceleration and velocity dynamic
    robot.setDynamicRel(0.5);

    std::array<double, 7> home = {M_PI_2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    auto motion_home = JointMotion(home);
    robot.move(motion_home);

    // Define and move forwards
    auto way = Affine(0.0, 0.2, 0.0);
    auto motion_forward = LinearRelativeMotion(way);
    robot.move(motion_forward);

    // And move backwards using the inverse motion
    auto motion_backward = LinearRelativeMotion(way.inverse());
    robot.move(motion_backward);

    return 0;
}
