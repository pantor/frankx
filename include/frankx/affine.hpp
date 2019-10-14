#pragma once

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <franka/robot.h>
#include <ReflexxesAPI.h>


struct Affine {
  using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  Eigen::Affine3d data;

  Affine() {
    this->data = Eigen::Affine3d::Identity();
  }

  Affine(const Eigen::Affine3d& data) {
    this->data = data;
  }


  Affine(double x, double y, double z, double a, double b, double c) {
    Eigen::EulerAnglesZYXd euler(a, b, c);
    data = Eigen::Translation<double, 3>(x, y, z) * euler.toRotationMatrix();
  }

  Affine(const Eigen::Matrix<double, 6, 1>& v): Affine(v(0), v(1), v(2), v(3), v(4), v(5)) { }

  Affine(const std::array<double, 16>& array) {
    Eigen::Affine3d affine(Eigen::Matrix4d::Map(array.data()));
    data = affine;
  }

  Affine(RMLVector<double> *rml_vector): Affine(rml_vector->VecData[0], rml_vector->VecData[1], rml_vector->VecData[2], rml_vector->VecData[3], rml_vector->VecData[4], rml_vector->VecData[5]) { }

  Affine(const franka::CartesianPose& pose, bool offset = true) {
    Eigen::EulerAnglesZYXd euler(M_PI_2, 0.0, M_PI);

    Eigen::Affine3d affine(Eigen::Matrix4d::Map(pose.O_T_EE.data()));
    const auto offset_affine = offset ? euler.toRotationMatrix() : Eigen::Affine3d::Identity().rotation();
    data = affine * offset_affine;
  }

  Affine operator *(const Affine &a) const {
    Eigen::Affine3d result;
    result = data * a.data;
    return Affine(result);
  }

  Affine inverse() const {
    return Affine(data.inverse());
  }

  bool isApprox(const Affine &a) const {
    return data.isApprox(a.data);
  }

  Eigen::Ref<Eigen::Affine3d::MatrixType> matrix() {
    return data.matrix();
  }

  void translate(const Eigen::Vector3d &v) {
    data.translate(v);
  }

  void pretranslate(const Eigen::Vector3d &v) {
    data.pretranslate(v);
  }

  Eigen::Vector3d translation() const {
    Eigen::Vector3d v;
    v << data.translation();
    return v;
  }

  double x() const {
    return data.translation()(0);
  }

  void set_x(double x) {
    data.translation()(0) = x;
  }

  double y() const {
    return data.translation()(1);
  }

  void set_y(double y) {
    data.translation()(1) = y;
  }

  double z() const {
    return data.translation()(2);
  }

  void set_z(double z) {
    data.translation()(2) = z;
  }

  void rotate(const Eigen::Affine3d::LinearMatrixType &r) {
    data.rotate(r);
  }

  void prerotate(const Eigen::Affine3d::LinearMatrixType &r) {
    data.prerotate(r);
  }

  Eigen::Affine3d::LinearMatrixType rotation() const {
    Eigen::Affine3d::LinearMatrixType result;
    result << data.rotation();
    return result;
  }

  double a() const {
    return static_cast<Euler>(data.rotation()).angles()(0);
  }

  double b() const {
    return static_cast<Euler>(data.rotation()).angles()(1);
  }

  double c() const {
    return static_cast<Euler>(data.rotation()).angles()(2);
  }

  void set_a(double a) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << static_cast<Euler>(data.rotation()).angles();
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(a, angles(1), angles(2)).toRotationMatrix();
  }

  void set_b(double b) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << static_cast<Euler>(data.rotation()).angles();
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(angles(0), b, angles(2)).toRotationMatrix();
  }

  void set_c(double c) {
    Eigen::Matrix<double, 3, 1> angles;
    angles << static_cast<Euler>(data.rotation()).angles();
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(angles(0), angles(1), c).toRotationMatrix();
  }

  Affine getInnerRandom() const {
    std::random_device r;
    std::default_random_engine engine(r());

    Eigen::Matrix<double, 6, 1> random;
    Eigen::Matrix<double, 6, 1> max;
    max << data.translation(), static_cast<Euler>(data.rotation()).angles();

    for (int i = 0; i < 6; i++) {
      std::uniform_real_distribution<double> distribution(-max(i), max(i));
      random(i) = distribution(engine);
    }

    return Affine(random);
  }

  std::string toString() const {
    Eigen::Matrix<double, 6, 1> v;
    v << data.translation(), static_cast<Euler>(data.rotation()).angles();

    return "[" + std::to_string(v(0)) + ", " + std::to_string(v(1)) + ", " + std::to_string(v(2))
      + ", " + std::to_string(v(3)) + ", " + std::to_string(v(4)) + ", " + std::to_string(v(5)) + "]";
  }
};