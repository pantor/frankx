#pragma once

#include <frankx/condition.hpp>
#include <frankx/utils.hpp>


namespace frankx {

struct MotionData {
    MotionData() { }

    double velocity_rel {1.0};
    double acceleration_rel {1.0};

    std::vector<Condition> conditions {};

    MotionData& withDynamics(double dynamics_rel) {
        this->velocity_rel = dynamics_rel;
        this->acceleration_rel = dynamics_rel;
        return *this;
    }

    MotionData& withCondition(Condition condition) {
        this->conditions.push_back(condition);
        return *this;
    }
};

} // namespace frankx