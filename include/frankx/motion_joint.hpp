#pragma once

#include <Eigen/Core>


namespace frankx {

struct JointMotion {
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d q_goal;

    explicit JointMotion(const std::array<double, 7> q_goal): q_goal(q_goal.data()) { }
};

} // namespace frankx
