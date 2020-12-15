#pragma once

#include <movex/path/path.hpp>


namespace movex {

struct Trajectory {
    struct State {
        //! The time i n[s]
        double t;

        //! The path position (between 0 and the path length) and its derivatives
        double s, ds, dds, ddds;
    };

    Path path;

    //! The trajectory state for each consecutive time step (with delta_time difference of the time parametrization)
    std::vector<State> states;

    explicit Trajectory() { }
    explicit Trajectory(const Path& path): path(path) { }
};

} // namespace movex
