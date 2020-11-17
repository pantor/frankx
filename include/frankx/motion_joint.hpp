#pragma once

#include <Eigen/Core>


namespace frankx {

struct JointMotion {
    const Vector7d target;

    explicit JointMotion(const std::array<double, 7> target): target(target.data()) { }
};

} // namespace frankx
