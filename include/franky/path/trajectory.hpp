#pragma once

#include "aggregated_path.hpp"


namespace franky {
  struct TrajectoryState {
    //! The time i n[s]
    double t;

    //! The path position (between 0 and the path length) and its derivatives
    double s, ds, dds, ddds;
  };

  template<typename PathType>
  struct Trajectory {
    PathType path;

    //! The trajectory state for each consecutive time step (with delta_time difference of the time parametrization)
    std::vector<TrajectoryState> states;

    explicit Trajectory() {}

    explicit Trajectory(const PathType &path) : path(path) {}
  };

} // namespace franky
