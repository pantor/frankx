#pragma once

#include <memory>

#include "franky/path/path.hpp"
#include "franky/waypoint.hpp"


namespace franky {
  template<size_t state_dimensions>
  class AggregatedPath : public Path<state_dimensions> {
    using Vector = Eigen::Matrix<double, state_dimensions, 1>;
  public:
    AggregatedPath() = default;

    AggregatedPath(const AggregatedPath<state_dimensions> &aggregated_path)
        : segments_(aggregated_path.segments_), cumulative_lengths_(aggregated_path.cumulative_lengths_) {}

    explicit AggregatedPath(const std::vector<std::shared_ptr<Path<state_dimensions>>> &segments)
        : segments_(segments) {
      if (segments_.empty())
        throw std::runtime_error("AggregatedPath needs at least one segment..");
      cumulative_lengths_.resize(segments_.size());
      double cumulative_length = 0.0;
      for (size_t i = 1; i < segments_.size() - 1; i += 1) {
        cumulative_length += segments_[i]->length();
        cumulative_lengths_[i] = cumulative_length;
      }
    }

    [[nodiscard]] inline double length() const override {
      return cumulative_lengths_.back();
    }

    Vector max_ddq() const override {
      Vector result;
      for (size_t i = 0; i < state_dimensions; i++) {
        auto max_ptr = *std::max_element(
            segments_.begin(), segments_.end(),
            [&](std::shared_ptr<Path<state_dimensions>> l, std::shared_ptr<Path<state_dimensions>> r) {
              return std::abs(l->max_ddq()(i)) < std::abs(r->max_ddq()(i));
            });
        result(i) = std::abs(max_ptr->max_ddq()(i));
      }
      return result;
    }

    Vector max_dddq() const override {
      Vector result;
      for (size_t i = 0; i < state_dimensions; i++) {
        auto max_ptr = *std::max_element(
            segments_.begin(), segments_.end(),
            [&](std::shared_ptr<Path<state_dimensions>> l, std::shared_ptr<Path<state_dimensions>> r) {
              return std::abs(l->max_dddq()(i)) < std::abs(r->max_dddq()(i));
            });
        result(i) = std::abs(max_ptr->max_dddq()(i));
      }
      return result;
    }

    PathStep<state_dimensions> operator()(double s) const override {
      size_t index = get_index(s);
      double s_local = (index == 0) ? s : s - cumulative_lengths_[index - 1];
      return (*segments_[index])(s_local);
    }

    [[nodiscard]] size_t get_index(double s) const {
      auto ptr = std::lower_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), s);
      size_t index = std::distance(cumulative_lengths_.begin(), ptr);
      return std::min(index, segments_.size() - 1);
    }

    inline std::vector<std::shared_ptr<Path<state_dimensions>>> segments() const {
      return segments_;
    }

  private:
    std::vector<std::shared_ptr<Path<state_dimensions>>> segments_;
    std::vector<double> cumulative_lengths_;
  };

  AggregatedPath<7>
  mk_path_from_waypoints(const std::vector<Waypoint> &waypoints, double default_initial_elbow_pos = 0.0);
}  // namespace franky
