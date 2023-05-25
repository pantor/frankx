#include "franky/path/aggregated_path.hpp"

#include "franky/waypoint.hpp"
#include "franky/path/linear_path.hpp"
#include "franky/path/quartic_blend_path.hpp"


namespace franky {
  AggregatedPath<7> mk_path_from_waypoints(
      const std::vector<Waypoint> &waypoints, double default_initial_elbow_pos) {
    if (waypoints.size() < 2) {
      throw std::runtime_error(
          "Path needs at least 2 waypoints as input, but has only " + std::to_string(waypoints.size()) + ".");
    }

    std::vector<std::shared_ptr<LinearPath<7>>> line_segments;
    RobotPose prev_robot_pose = waypoints[0].robot_pose;
    if (!prev_robot_pose.elbow_position().has_value())
      prev_robot_pose = prev_robot_pose.with_elbow_position(default_initial_elbow_pos);

    for (size_t i = 1; i < waypoints.size(); i += 1) {
      auto segment = std::make_shared<LinearPath<7>>(
          prev_robot_pose.vector_repr(), waypoints[i].robot_pose.vector_repr());
      line_segments.emplace_back(segment);
      prev_robot_pose = waypoints[i].robot_pose;
    }

    std::vector<std::shared_ptr<Path<7>>> segments;

    for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
      if (waypoints[i].blend_max_distance > 0.0) {
        auto &left = line_segments[i - 1];
        auto &right = line_segments[i];

        Vector7d lm = (left->end() - left->start()) / left->length();
        Vector7d rm = (right->end() - right->start()) / right->length();

        auto s_abs_max = std::min<double>({left->length() / 2, right->length() / 2});

        auto blend = std::make_shared<QuarticBlendPath<7>>(
            left->start(), lm, rm, left->length(), waypoints[i].blend_max_distance, s_abs_max);
        double s_abs = blend->length() / 2;

        auto new_left = std::make_shared<LinearPath<7>>(
            left->start(), (*left)(left->length() - s_abs).q);
        auto new_right = std::make_shared<LinearPath<7>>((*right)(s_abs).q, right->end());

        segments.emplace_back(new_left);
        segments.emplace_back(blend);
        right = new_right;
      } else {
        segments.emplace_back(line_segments[i - 1]);
      }
    }
    segments.emplace_back(line_segments.back());
    return AggregatedPath<7>(segments);
  }
}  // namespace franky
