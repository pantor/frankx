#pragma once

#include <Eigen/Core>


namespace movex {

/**
* A motion in the joint space
*/
struct JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d target;

    explicit JointMotion(const std::array<double, 7> target): target(target.data()) { }
};

} // namespace movex
