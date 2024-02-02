#define CATCH_CONFIG_MAIN
#include <random>

#include <catch2/catch.hpp>
#include <Eigen/Core>

#include <franky/robot_pose.hpp>
#include <franky/path/path.hpp>
#include <franky/path/time_parametrization.hpp>

using namespace franky;

void check_path(const std::vector<PathWaypoint> &waypoints, double blend_max_distance) {
  CAPTURE(waypoints);
  CAPTURE(blend_max_distance);

  if (waypoints.size() < 2) {
    CHECK_THROWS(mk_path_from_waypoints(waypoints, blend_max_distance));
    return;
  }

  auto path = mk_path_from_waypoints(waypoints, blend_max_distance);
  auto tp = TimeParametrization(0.001);

  CHECK(path.length() >= 0.0);

  auto max_velocity = std::array<double, 7>{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  auto max_acceleration = std::array<double, 7>{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  auto max_jerk = std::array<double, 7>{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};

  auto trajectory = tp.parametrize(path, max_velocity, max_acceleration, max_jerk);
}

TEST_CASE("Path from affines and blending") {
  srand(44);
  std::default_random_engine gen;
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (size_t i = 0; i < 1024; i += 1) {
    size_t n = 2 + 4 * dist(gen);
    if (i < 2) { // Make a few examples with too few waypoints to test exceptions
      n = i;
    }

    std::vector<PathWaypoint> waypoints(n);
    for (size_t j = 0; j < n; j += 1) {
      auto translation = Eigen::Vector3d::Random();
      auto quaternion = Eigen::Vector4d::Random().normalized();
      auto transformation = Affine().fromPositionOrientationScale(
          translation, Eigen::Quaterniond(quaternion), Eigen::Vector3d::Ones());
      waypoints[j] = PathWaypoint{{transformation}};
    }
    double blend_max = 0.1 * dist(gen);

    check_path(waypoints, blend_max);
  }
}
