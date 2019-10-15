#pragma once

#include <random>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <franka/robot.h>
#include <ReflexxesAPI.h>


namespace frankx {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

struct Affine {
    using Euler = Eigen::EulerAnglesZYXd;

    Eigen::Affine3d data {};
    Eigen::Vector3d ref_euler {Eigen::Vector3d::Zero()};

    Euler offset_euler {M_PI_2, 0.0, M_PI};

    explicit Affine();
    explicit Affine(const Eigen::Affine3d& data);
    explicit Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0);
    explicit Affine(const Vector6d& v);
    explicit Affine(const Vector7d& v);
    explicit Affine(const std::array<double, 16>& array);
    explicit Affine(RMLVector<double> *rml_vector);
    explicit Affine(const franka::CartesianPose& pose, bool offset = true);

    Affine operator*(const Affine &a) const;

    Affine inverse() const;

    bool isApprox(const Affine &a) const;

    Eigen::Ref<Eigen::Affine3d::MatrixType> matrix();
    std::array<double, 16> array() const;

    void translate(const Eigen::Vector3d &v);
    void pretranslate(const Eigen::Vector3d &v);
    void rotate(const Eigen::Affine3d::LinearMatrixType &r);
    void prerotate(const Eigen::Affine3d::LinearMatrixType &r);

    Eigen::Vector3d translation() const;
    Eigen::Vector3d angles() const;
    Eigen::Vector3d angles(const Eigen::Vector3d& new_ref_euler);
    Eigen::Affine3d::LinearMatrixType rotation() const;

    Vector6d vector() const;
    Vector6d vector(const Eigen::Vector3d& new_ref_euler);
    Vector7d vector(double elbow) const;
    Vector7d vector(double elbow, const Eigen::Vector3d& new_ref_euler);
    Vector7d vector(double elbow, const Vector7d& new_ref_vector);

    double x() const;
    double y() const;
    double z() const;

    void set_x(double x);
    void set_y(double y);
    void set_z(double z);

    double a() const;
    double b() const;
    double c() const;

    void set_a(double a);
    void set_b(double b);
    void set_c(double c);

    Affine getInnerRandom() const;
    std::string toString() const;
};

} // namespace frankx
