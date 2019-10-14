#include <iostream>
#include <iterator>

#include <frankx/gripper.hpp>
#include <frankx/motion_data.hpp>
#include <frankx/motion_joint.hpp>
#include <frankx/motion_waypoint.hpp>
#include <frankx/robot.hpp>


inline Eigen::Affine3d getBase(double x = 0.0, double y = 0.0, double z = 0.0, double a = 0.0, double b = 0.0, double c = 0.0) {
  return Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c);
}


int main() {
  frankx::Robot robot("172.16.0.2");
  robot.automaticErrorRecovery();
  robot.setDefault();


  JointMotion joint_motion(0.2, {-1.8119446041276, 1.1791089121678, 1.7571002245448, -2.141621800118, -1.1433693913722, 1.6330460616663, -0.4321716643888});
  robot.move(joint_motion);

  auto motion = LinearRelativeMotion(Affine(0.0, 0.0, -0.12, 0.0, 0.0, 0.0), -0.2);
  auto data = MotionData().withDynamics(0.5).withCondition(Condition(Condition::Axis::ForceZ, Condition::Comparison::Smaller, -7.0, std::make_shared<LinearRelativeMotion>(Affine(0.0, 0.0, 0.001, 0.0, 0.0, 0.0), 0.0)));
  robot.move(motion, data);


  // WaypointMotion waypoint_motion({
  //   Waypoint(getBase(0.05, 0.05, 0.0, M_PI_2), 1.75),
  //   Waypoint(getBase(0.05, 0.05, -0.185, M_PI_2), 1.35)
  // });
  // robot.move(waypoint_motion);


  // WaypointMotion waypoint_motion2({
  //   Waypoint(getBase(0.05, 0.05, 0.0, M_PI_2), 1.75),
  //   Waypoint(getBase(0.0, 0.0, 0.0), 1.75)
  // });
  // robot.move(waypoint_motion2);

  return 0;
}
