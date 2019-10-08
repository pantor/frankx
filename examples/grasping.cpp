#include <iostream>
#include <iterator>

#include <frankx/geometry.hpp>
#include <frankx/gripper.hpp>
#include <frankx/motion_joint.hpp>
#include <frankx/motion_waypoint.hpp>
#include <frankx/robot.hpp>
#include <frankx/waypoint.hpp>



inline Eigen::Affine3d getRelativeBase(double x = 0.0, double y = 0.0, double z = 0.0, double a = 0.0, double b = 0.0, double c = 0.0) {
  return Affine(0.48 + x, -0.204 + y, 0.267 + z, M_PI_2 + a, 0.0 + b, M_PI + 1e-9 + c);
}


int main() {
  frankx::Robot robot("172.16.0.2");
  robot.automaticErrorRecovery();
  robot.setDefault();


  JointMotion joint_motion(0.3, {-1.8119446041276943, 1.1791089121678338, 1.7571002245448795, -2.141621800118565, -1.1433693913722132, 1.6330460616663416, -0.4321716643888135});
  robot.control(joint_motion);


  WaypointMotion waypoint_motion({
    Waypoint(getRelativeBase(0.05, 0.05, 0.0, M_PI_2), 1.6),
    // Waypoint(getRelativeBase(0.05, 0.05, -0.185, M_PI_2), 1.35)
  });
  robot.move(waypoint_motion);


  WaypointMotion waypoint_motion2({
    Waypoint(getRelativeBase(0.05, 0.05, 0.0, M_PI_2 - 2 * M_PI), 1.6),
    // Waypoint(getRelativeBase(0.05, 0.05, -0.185, M_PI_2), 1.35)
  });
  robot.move(waypoint_motion2);


  return 0;
}
