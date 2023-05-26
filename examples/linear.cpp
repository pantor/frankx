#include <franky.hpp>

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
  auto motion_forward = std::make_shared<LinearMotion>(RobotPose(way), true);
  robot.move(motion_forward);

  // And move backwards using the inverse motion
  auto motion_backward = std::make_shared<LinearMotion>(RobotPose(way.inverse()), true);
  robot.move(motion_backward);

  return 0;
}
