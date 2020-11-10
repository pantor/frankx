#pragma once

#include <Eigen/Core>

#include <otgx/parameter.hpp>


namespace otgx {

template<size_t DOFs>
class Ruckig {
    InputParameter<DOFs> current_input;

public:
    double delta_time;

    explicit Ruckig(double delta_time): delta_time(delta_time) {

    }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        if (input != current_input) {
            current_input = input;
        }
    }
};

} // namespace otgx
