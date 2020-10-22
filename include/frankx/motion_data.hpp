#pragma once

#include <frankx/reaction.hpp>


namespace frankx {


struct MotionData {
    explicit MotionData(double dynamic_rel = 1.0): velocity_rel(dynamic_rel), acceleration_rel(dynamic_rel), jerk_rel(dynamic_rel) { }

    double velocity_rel {1.0}, acceleration_rel {1.0}, jerk_rel {1.0};
    std::vector<Reaction> reactions {};

    MotionData& withDynamicRel(double dynamic_rel) {
        velocity_rel = dynamic_rel;
        acceleration_rel = dynamic_rel;
        jerk_rel = dynamic_rel;
        return *this;
    }

    MotionData& withReaction(const Reaction& reaction) {
        reactions.push_back(reaction);
        return *this;
    }
};

} // namespace frankx