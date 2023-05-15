#pragma once

#include <array>
#include <random>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>


namespace franky {

class Affine {
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  using Euler = Eigen::EulerAngles<double, Eigen::EulerSystemZYX>;
  typedef Eigen::Affine3d Type;

public:
  Type data;

  explicit Affine() {
    this->data = Type::Identity();
  }

  explicit Affine(const Type& data) {
    this->data = data;
  }

  explicit Affine(double x, double y, double z, double a = 0.0, double b = 0.0, double c = 0.0) {
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Euler(a, b, c).toRotationMatrix();
  }

  explicit Affine(double x, double y, double z, double q_w, double q_x, double q_y, double q_z) {
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Eigen::Quaterniond(q_w, q_x, q_y, q_z).toRotationMatrix();
  }

  explicit Affine(const std::array<double, 6>& v): Affine(v[0], v[1], v[2], v[3], v[4], v[5]) { }

  explicit Affine(const std::array<double, 7>& v): Affine(v[0], v[1], v[2], v[3], v[4], v[5]) { }

  explicit Affine(const Vector6d& v): Affine(v[0], v[1], v[2], v[3], v[4], v[5]) { }

  explicit Affine(const Vector7d& v): Affine(v[0], v[1], v[2], v[3], v[4], v[5]) { }

  explicit Affine(const std::array<double, 16>& array) {
    Type affine(Eigen::Matrix4d::Map(array.data()));
    data = affine;
  }

  static Affine fromPosRotVec(double p_x, double p_y, double p_z, double r_x, double r_y, double r_z) {
      auto output = Affine();
      output.data.translation() = Eigen::Vector3d(p_x, p_y, p_z);
      auto rot_vec = Eigen::Vector3d(r_x, r_y, r_z);
      auto axis = rot_vec.normalized();
      auto angle = rot_vec.norm();
      output.data.linear() = Eigen::AngleAxis(angle, axis).toRotationMatrix();
      return output;
  }

  static Affine fromPosRotVec(const std::array<double, 6> &v) {
      return fromPosRotVec(v[0], v[1], v[2], v[3], v[4], v[5]);
  }

  static Affine fromPosRotVec(const std::array<double, 7> &v) {
      return fromPosRotVec(v[0], v[1], v[2], v[3], v[4], v[5]);
  }

  static Affine fromPosRotVec(const Vector6d &v) {
      return fromPosRotVec(v[0], v[1], v[2], v[3], v[4], v[5]);
  }

  static Affine fromPosRotVec(const Vector7d &v) {
      return fromPosRotVec(v[0], v[1], v[2], v[3], v[4], v[5]);
  }

  Affine operator *(const Affine &a) const {
    Type result;
    result = data * a.data;
    return Affine(result);
  }

  Affine inverse() const {
    return Affine(data.inverse());
  }

  bool isApprox(const Affine &a) const {
    return data.isApprox(a.data);
  }

  Eigen::Ref<Affine::Type::MatrixType> matrix() {
    return data.matrix();
  }

  std::array<double, 16> array() const {
    std::array<double, 16> array;
    std::copy(data.data(), data.data() + array.size(), array.begin());
    return array;
  }

  Vector6d vector() const {
    Vector6d result;
    result << data.translation(), angles();
    return result;
  }

  Eigen::Vector3d rotation_vector() const {
      Eigen::AngleAxis<double> result(data.rotation());
      return result.axis() * result.angle();
  }

  Vector7d vector_with_elbow(double elbow) const {
    Vector7d result;
    result << data.translation(), rotation_vector(), elbow;
    return result;
  }

  Eigen::Vector3d translation() const {
    Eigen::Vector3d v;
    v << data.translation();
    return v;
  }

  Eigen::Vector3d angles() const {
    Eigen::Vector3d angles = Euler(data.rotation()).angles();
    Eigen::Vector3d angles_equal;
    angles_equal << angles[0] - M_PI, M_PI - angles[1], angles[2] - M_PI;

    if (angles_equal[1] > M_PI) {
      angles_equal[1] -= 2 * M_PI;
    }
    if (angles_equal[2] < -M_PI) {
      angles_equal[2] += 2 * M_PI;
    }

    if (angles.norm() < angles_equal.norm()) {
      return angles;
    }
    return angles_equal;
  }

  Type::LinearMatrixType rotation() const {
    Type::LinearMatrixType result;
    result << data.rotation();
    return result;
  }

  Eigen::Quaterniond quaternion() const {
    Eigen::Quaterniond q(data.rotation());
    return q;
  }

  std::array<double, 4> py_quaternion() const {
    auto q = quaternion();
    return {q.w(), q.x(), q.y(), q.z()};
  }

  double x() const {
    return data.translation().x();
  }

  double y() const {
    return data.translation().y();
  }

  double z() const {
    return data.translation().z();
  }

  double a() const {
    return angles()(0);
  }

  double b() const {
    return angles()(1);
  }

  double c() const {
    return angles()(2);
  }

  double qW() const {
    return quaternion().w();
  }

  double qX() const {
    return quaternion().x();
  }

  double qY() const {
    return quaternion().y();
  }

  double qZ() const {
    return quaternion().z();
  }

  void translate(const Eigen::Vector3d &v) {
    data.translate(v);
  }

  void pretranslate(const Eigen::Vector3d &v) {
    data.pretranslate(v);
  }

  void rotate(const Type::LinearMatrixType &r) {
    data.rotate(r);
  }

  void prerotate(const Type::LinearMatrixType &r) {
    data.prerotate(r);
  }

  void setQuaternion(double w, double x, double y, double z) {
    data.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  }

  void setX(double x) {
    data.translation().x() = x;
  }

  void setY(double y) {
    data.translation().y() = y;
  }

  void setZ(double z) {
    data.translation().z() = z;
  }

  void setA(double a) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(a, euler(1), euler(2)).toRotationMatrix();
  }

  void setB(double b) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(euler(0), b, euler(2)).toRotationMatrix();
  }

  void setC(double c) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(euler(0), euler(1), c).toRotationMatrix();
  }

  Affine slerp(const Affine& affine, double t) const {
    Type result;
    Eigen::Quaterniond q_start(data.rotation());
    Eigen::Quaterniond q_end(affine.rotation());
    result.translation() = data.translation() + t * (affine.translation() - data.translation());
    result.linear() = q_start.slerp(t, q_end).toRotationMatrix();
    return Affine(result);
  }

  Affine getInnerRandom() const {
    std::random_device r;
    std::default_random_engine engine(r());

    Eigen::Matrix<double, 6, 1> max, random;
    max << data.translation(), angles();

    for (int i = 0; i < 6; i++) {
      std::uniform_real_distribution<double> distribution(-max(i), max(i));
      random(i) = distribution(engine);
    }

    return Affine(random(0), random(1), random(2), random(3), random(4), random(5));
  }

  std::string toString() const {
    Eigen::Matrix<double, 6, 1> v;
    v << data.translation(), angles();

    return "[" + std::to_string(v(0)) + ", " + std::to_string(v(1)) + ", " + std::to_string(v(2))
      + ", " + std::to_string(v(3)) + ", " + std::to_string(v(4)) + ", " + std::to_string(v(5)) + "]";
  }
};

} // namespace franky
