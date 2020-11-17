#include <frankx/frankx.hpp>


using namespace frankx;

inline Affine getBase(double x = 0.0, double y = 0.0, double z = 0.0, double a = 0.0, double b = 0.0, double c = 0.0) {
    return Affine(0.48 + x, -0.204 + y, 0.267 + z, a, b, c);
}


int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    Robot robot(argv[1]);
    robot.automaticErrorRecovery();
    robot.setDefaultBehavior();

    auto joint_motion = JointMotion({-1.8119446041, 1.1791089121, 1.7571002245, -2.141621800, -1.1433693913, 1.6330460616, -0.4321716643});
    auto joint_data = MotionData().withDynamicRel(0.2);
    robot.move(joint_motion, joint_data);

    auto data = MotionData().withDynamicRel(0.5).withReaction(Reaction(
        Measure::ForceZ() < -7.0,
        std::make_shared<LinearRelativeMotion>(Affine(0.0, 0.0, 0.001))
    ));

    WaypointMotion waypoint_motion({
        Waypoint {getBase(0.05, 0.05, 0.0, M_PI_2)},
        Waypoint {getBase(0.05, 0.05, -0.185, M_PI_2)}
    });
    robot.move(waypoint_motion, data);

    WaypointMotion waypoint_motion2({
        Waypoint {getBase(0.05, 0.05, 0.0, M_PI_2)},
        Waypoint {getBase(0.0, 0.0, 0.0), 1.75}
    });
    robot.move(waypoint_motion2);

    return 0;
}
