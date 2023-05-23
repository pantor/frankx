#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "franky/types.hpp"


namespace franky {

  struct Kinematics {
    template<class MatT>
    static Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
    pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) {
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

      explicit NullSpaceHandling(size_t joint_index, double value) : joint_index(joint_index), value(value) {}
    };

    static Affine forward(const Eigen::Matrix<double, 7, 1> &q);

    static Affine forwardElbow(const Eigen::Matrix<double, 7, 1> &q);

    static Eigen::Matrix<double, 6, 1> forwardEuler(const Eigen::Matrix<double, 7, 1> &q);

    static Eigen::Matrix<double, 6, 7> jacobian(const Eigen::Matrix<double, 7, 1> &q);

    static Eigen::Matrix<double, 7, 1>
    inverse(const Eigen::Matrix<double, 6, 1> &x_target, const Eigen::Matrix<double, 7, 1> &q0,
            std::optional<NullSpaceHandling> null_space = std::nullopt);
  };

  template<size_t DoFs>
  class KinematicChain {
    struct DenavitHartenbergParameter {
      double alpha; // [rad]
      double d, a; // [m]

      Affine get_transformation(double theta) const {
        Affine rot1;
        rot1.linear() = Euler(theta, 0, 0).toRotationMatrix();
        Affine trans1;
        trans1.affine() = Eigen::Vector3d(0, 0, d);
        Affine trans2;
        trans2.affine() = Eigen::Vector3d(a, 0, 0);
        Affine rot2;
        rot2.linear() = Euler(0, 0, alpha).toRotationMatrix();
        return rot2 * trans2 * rot1 * trans1;
      }
    };

    std::array<DenavitHartenbergParameter, DoFs> parameters;
    Affine base;

  public:
    explicit KinematicChain(const std::array<DenavitHartenbergParameter, DoFs> &parameters, const Affine &base)
        : parameters(parameters), base(base) {}

    Affine forward_chain(const std::array<double, DoFs> &q) const {
      Affine result;
      for (size_t i = 0; i < DoFs; ++i) {
        result = result * parameters[i].get_transformation(q[i]);
      }
      return result * base;
    }
  };

}
