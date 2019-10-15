#include <iostream>
#include <iterator>

#include <frankx/gripper.hpp>
#include <frankx/motion_data.hpp>
#include <frankx/motion_joint.hpp>
#include <frankx/motion_waypoint.hpp>
#include <frankx/robot.hpp>


// using namespace frankx;

inline frankx::Affine getBase(double x = 0.0, double y = 0.0, double z = 0.0, double a = 0.0, double b = 0.0, double c = 0.0) {
  return frankx::Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c);
}


int main() {
  frankx::Robot robot("172.16.0.2");
  robot.automaticErrorRecovery();
  robot.setDefault();


  frankx::JointMotion joint_motion(0.2, {-1.8119446041, 1.1791089121, 1.7571002245, -2.141621800, -1.1433693913, 1.6330460616, -0.4321716643});
  robot.move(joint_motion);

  auto motion = frankx::LinearRelativeMotion(frankx::Affine(0.0, 0.0, -0.12, 0.0, 0.0, 0.0), -0.2);
  auto data = frankx::MotionData().withDynamics(0.5).withCondition(
    frankx::Condition(
      frankx::Condition::Measure::ForceZ,
      frankx::Condition::Comparison::Smaller,
      -7.0,
      std::make_shared<frankx::LinearRelativeMotion>(frankx::Affine(0.0, 0.0, 0.001, 0.0, 0.0, 0.0), 0.0)
    )
  );
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
