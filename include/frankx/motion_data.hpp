#pragma once

#include <frankx/condition.hpp>
#include <frankx/utils.hpp>


namespace frankx {

struct MotionData {
    explicit MotionData(double dynamic_rel = 1.0): velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) { }

    double velocity_rel;
    double acceleration_rel;
    double jerk_rel;

    std::vector<Condition> conditions {};

    MotionData& withDynamicRel(double dynamic_rel) {
        this->velocity_rel = dynamic_rel;
        this->acceleration_rel = dynamic_rel;
        this->jerk_rel = dynamic_rel;
        return *this;
    }

    MotionData& withCondition(Condition condition) {
        this->conditions.push_back(condition);
        return *this;
    }
};

} // namespace frankx