#pragma once

#include <random>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


namespace movex {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

struct Affine {
    using Euler = Eigen::EulerAnglesZYXd;
    typedef Eigen::Affine3d Type;  // Isometry3d would be preferred, but implementation does not guarantee to remain an isometry

    Type data {};

    explicit Affine();
    explicit Affine(const Type& data);
    explicit Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0);
    explicit Affine(double x, double y, double z, double q_w, double q_x, double q_y, double q_z);
    explicit Affine(const Vector6d& v);
    explicit Affine(const Vector7d& v);
    explicit Affine(const std::array<double, 16>& array);

    Affine operator*(const Affine &a) const;
    Affine inverse() const;
    bool isApprox(const Affine &a) const;
    Eigen::Ref<Type::MatrixType> matrix();
    std::array<double, 16> array() const;

    void translate(const Eigen::Vector3d &v);
    void pretranslate(const Eigen::Vector3d &v);
    void rotate(const Type::LinearMatrixType &r);
    void prerotate(const Type::LinearMatrixType &r);

    Eigen::Vector3d translation() const;
    Eigen::Vector3d angles() const;
    Type::LinearMatrixType rotation() const;
    Eigen::Quaterniond quaternion() const;

    Vector6d vector() const;
    Vector7d vector_with_elbow(double elbow) const;

    double x() const;
    double y() const;
    double z() const;
    double a() const;
    double b() const;
    double c() const;
    double q_w() const;
    double q_x() const;
    double q_y() const;
    double q_z() const;

    void set_x(double x);
    void set_y(double y);
    void set_z(double z);
    void set_a(double a);
    void set_b(double b);
    void set_c(double c);
    void set_quaternion(double w, double x, double y, double z);

    Affine slerp(const Affine& affine, double t) const;
    Affine getInnerRandom() const;
    std::string toString() const;
};

} // namespace movex
