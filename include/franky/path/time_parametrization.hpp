#pragma once

#include <ruckig/ruckig.hpp>

#include "franky/path/trajectory.hpp"
#include "franky/types.hpp"


namespace franky {
  class TimeParametrization {
  private:
    const std::unique_ptr<ruckig::Ruckig<1>> otg;

    //! Time step between updates (cycle time) in [s]
    const double delta_time;

  public:
    explicit TimeParametrization(double delta_time)
        : delta_time(delta_time), otg(std::make_unique<ruckig::Ruckig<1>>(delta_time)) {}

    //! Returns list of path positions s at delta time
    template<size_t state_dimensions>
    Trajectory<AggregatedPath<state_dimensions>> parametrize(
        const AggregatedPath<state_dimensions> &path,
        const std::array<double, state_dimensions> &max_velocity,
        const std::array<double, state_dimensions> &max_acceleration,
        const std::array<double, state_dimensions> &max_jerk) {
      Trajectory trajectory{path};

      // For linear segments: accelerate as fast as possible
      // For blend segments: Constant path velocity ds
      // Check continuous, and go back to zero velocity otherwise

      Vector<state_dimensions> max_velocity_v = Eigen::Map<const Vector<state_dimensions>>(
          max_velocity.data(), max_velocity.size());
      Vector<state_dimensions> max_accleration_v = Eigen::Map<const Vector<state_dimensions>>(
          max_acceleration.data(), max_acceleration.size());
      Vector<state_dimensions> max_jerk_v = Eigen::Map<const Vector<state_dimensions>>(
          max_jerk.data(), max_jerk.size());

      std::vector<std::tuple<double, double, double>> max_path_dynamics;
      for (auto segment: path.segments()) {
        auto max_pddq = segment->max_ddq();
        auto max_pdddq = segment->max_dddq();

        double max_ds, max_dds, max_ddds;

        // Linear segments
        if ((max_pddq.array().abs() < 1e-16).any() && (max_pdddq.array().abs() < 1e-16).any()) {
          auto constant_pdq = (*segment)(0.0).dq;

          max_ds = (max_velocity_v.array() / constant_pdq.array().abs()).minCoeff();
          max_dds = (max_accleration_v.array() / constant_pdq.array().abs()).minCoeff();
          max_ddds = (max_jerk_v.array() / constant_pdq.array().abs()).minCoeff();

          // Other segments
        } else {
          // ds = max_velocity_v.array() / pdq(s)  // pdq will always be between two linear segments...
          double ds_acc = (max_accleration_v.array() / max_pddq.array().abs()).sqrt().minCoeff();
          double ds_jerk = (max_jerk_v.array() / max_pdddq.array().abs()).pow(1. / 3).minCoeff();
          max_ds = std::min(ds_acc, ds_jerk);
          max_dds = 0.0;
          max_ddds = 0.0;
        }

        max_path_dynamics.emplace_back(max_ds, max_dds, max_ddds);
      }

      // Integrate forward and (if necessary) backward
      // Get maximal possible velocity, acceleration

      // Calculate path at time steps
      ruckig::InputParameter<1> input;
      ruckig::OutputParameter<1> output;
      ruckig::Result otg_result{ruckig::Result::Working};

      double time{0.0};
      double s_new{0.0}, ds_new{0.0}, dds_new{0.0};
      size_t index_current = path.get_index(s_new);

      trajectory.states.emplace_back({time, s_new, ds_new, dds_new, 0.0});

      input.current_position[0] = s_new;
      input.current_velocity[0] = ds_new;
      input.current_acceleration[0] = dds_new;
      input.target_position[0] = path.length();
      input.target_velocity[0] = 0.0;
      std::tie(input.max_velocity[0], input.max_acceleration[0], input.max_jerk[0]) = max_path_dynamics[index_current];

      while (otg_result == ruckig::Result::Working) {
        time += delta_time;
        otg_result = otg->update(input, output);

        s_new = output.new_position[0];
        ds_new = output.new_velocity[0];
        dds_new = output.new_acceleration[0];

        size_t index_new = path.get_index(s_new);

        // New segment
        if (index_new > index_current) {
          index_current = index_new;
        }

        trajectory.states.emplace_back({time, s_new, ds_new, dds_new, 0.0});
        output.pass_to_input(input);
      }

      return trajectory;
    }
  };
} // namespace franky
