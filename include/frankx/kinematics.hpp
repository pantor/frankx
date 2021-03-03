#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>


namespace frankx {

struct Kinematics {
    template <class MatT>
    static Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) {
        typedef typename MatT::Scalar Scalar;
        auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        const auto &singularValues = svd.singularValues();
        Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
        singularValuesInv.setZero();
        for (unsigned int i = 0; i < singularValues.size(); ++i) {
            if (singularValues(i) > tolerance) {
                singularValuesInv(i, i) = Scalar{1} / singularValues(i);

            } else {
                singularValuesInv(i, i) = Scalar{0};
            }
        }
        return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }

    struct NullSpaceHandling {
        size_t joint_index;
        double value;

        explicit NullSpaceHandling(size_t joint_index, double value): joint_index(joint_index), value(value) {}
    };

    static std::array<double, 16> forward(const Eigen::Matrix<double, 7, 1>& q);
    static std::array<double, 16> forwardElbow(const Eigen::Matrix<double, 7, 1>& q);
    static Eigen::Matrix<double, 6, 1> forwardEuler(const Eigen::Matrix<double, 7, 1>& q);
    static Eigen::Matrix<double, 6, 7> jacobian(const Eigen::Matrix<double, 7, 1>& q);
    static Eigen::Matrix<double, 7, 1> inverse(const Eigen::Matrix<double, 6, 1>& x_target, const Eigen::Matrix<double, 7, 1>& q0, std::optional<NullSpaceHandling> null_space = std::nullopt);
};

}
