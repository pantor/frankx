#pragma once

#include <optional>

#include <Eigen/Core>


namespace movex {

enum class Result {
    Working,
    Finished,
    Error
};


template<size_t DOFs>
struct InputParameter {
    using Vector = Eigen::Matrix<double, DOFs, 1, Eigen::ColMajor>;
    static constexpr size_t degrees_of_freedom {DOFs};

    Vector current_position;
    Vector current_velocity {Vector::Zero()};
    Vector current_acceleration {Vector::Zero()};

    Vector target_position;
    Vector target_velocity {Vector::Zero()};
    Vector target_acceleration {Vector::Zero()};

    Vector max_velocity;
    Vector max_acceleration;
    Vector max_jerk;

    std::array<bool, DOFs> enabled;
    std::optional<double> minimum_duration;

    InputParameter() {
        enabled.fill(true);
    }

    bool operator!=(const InputParameter<DOFs>& rhs) const {
        return (
            current_position != rhs.current_position
            || current_velocity != rhs.current_velocity
            || current_acceleration != rhs.current_acceleration
            || target_position != rhs.target_position
            || target_velocity != rhs.target_velocity
            || target_acceleration != rhs.target_acceleration
            || max_velocity != rhs.max_velocity
            || max_acceleration != rhs.max_acceleration
            || max_jerk != rhs.max_jerk
            || enabled != enabled
            || minimum_duration != minimum_duration
        );
    }
};


template<size_t DOFs>
struct OutputParameter {
    using Vector = Eigen::Matrix<double, DOFs, 1>;

    Vector new_position;
    Vector new_velocity;
    Vector new_acceleration;

    double duration;
};

} // namespace movex
