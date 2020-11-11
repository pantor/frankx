#include <frankx/affine.hpp>


namespace frankx {

Affine::Affine() {
    this->data = Type::Identity();
}

Affine::Affine(const Type& data) {
    this->data = data;
}

Affine::Affine(double x, double y, double z, double a, double b, double c) {
    // data = Eigen::Translation<double, 3>(x, y, z) * Euler(a, b, c).toRotationMatrix();
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Euler(a, b, c).toRotationMatrix();
}

Affine::Affine(double x, double y, double z, double q_w, double q_x, double q_y, double q_z) {
    data.translation() = Eigen::Vector3d(x, y, z);
    data.linear() = Eigen::Quaterniond(q_w, q_x, q_y, q_z).toRotationMatrix();
}

Affine::Affine(const Vector6d& v): Affine(v(0), v(1), v(2), v(3), v(4), v(5)) { }

Affine::Affine(const Vector7d& v): Affine(v(0), v(1), v(2), v(3), v(4), v(5)) { }

Affine::Affine(const std::array<double, 16>& array) {
    Type affine(Eigen::Matrix4d::Map(array.data()));
    data = affine;
}

#ifdef AFFINE_WITH_ROBOT_CONTROL

Affine::Affine(const franka::CartesianPose& pose) {
    Type affine(Eigen::Matrix4d::Map(pose.O_T_EE.data()));
    data = affine;
}

#endif

Affine Affine::operator *(const Affine &a) const {
    Type result;
    result = data * a.data;
    return Affine(result);
}

Affine Affine::inverse() const {
    return Affine(data.inverse());
}

bool Affine::isApprox(const Affine &a) const {
    return data.isApprox(a.data);
}

Eigen::Ref<Affine::Type::MatrixType> Affine::matrix() {
    return data.matrix();
}

void Affine::translate(const Eigen::Vector3d &v) {
    data.translate(v);
}

void Affine::pretranslate(const Eigen::Vector3d &v) {
    data.pretranslate(v);
}

Eigen::Vector3d Affine::translation() const {
    Eigen::Vector3d v;
    v << data.translation();
    return v;
}

Eigen::Vector3d Affine::angles() const {
    Eigen::Vector3d euler = Euler::FromRotation<false, false, false>(data.rotation()).angles();
    Eigen::Vector3d euler2;
    euler2 << euler[0] - M_PI, M_PI - euler[1], euler[2] - M_PI;

    if (euler2[1] > M_PI) {
        euler2[1] -= 2 * M_PI;
    }
    if (euler2[2] < -M_PI) {
        euler2[2] += 2 * M_PI;
    }

    return (euler.norm() < euler2.norm()) ? euler : euler2;
}

Vector6d Affine::vector() const {
    Vector6d result;
    result << data.translation(), angles();
    return result;
}

Vector7d Affine::vector_with_elbow(double elbow) const {
    Vector7d result;
    result << data.translation(), angles(), elbow;
    return result;
}

std::array<double, 16> Affine::array() const {
    std::array<double, 16> array {};
    std::copy( data.data(), data.data() + array.size(), array.begin() );
    return array;
}

double Affine::x() const {
    return data.translation()(0);
}

void Affine::set_x(double x) {
    data.translation()(0) = x;
}

double Affine::y() const {
    return data.translation()(1);
}

void Affine::set_y(double y) {
    data.translation()(1) = y;
}

double Affine::z() const {
    return data.translation()(2);
}

void Affine::set_z(double z) {
    data.translation()(2) = z;
}

void Affine::rotate(const Type::LinearMatrixType &r) {
    data.rotate(r);
}

void Affine::prerotate(const Type::LinearMatrixType &r) {
    data.prerotate(r);
}

Affine::Type::LinearMatrixType Affine::rotation() const {
    Type::LinearMatrixType result;
    result << data.rotation();
    return result;
}

Eigen::Quaterniond Affine::quaternion() const {
    Eigen::Quaterniond q(data.rotation());
    return q;
}

double Affine::a() const {
    return angles()(0);
}

double Affine::b() const {
    return angles()(1);
}

double Affine::c() const {
    return angles()(2);
}

void Affine::set_a(double a) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(a, euler(1), euler(2)).toRotationMatrix();
}

void Affine::set_b(double b) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(euler(0), b, euler(2)).toRotationMatrix();
}

void Affine::set_c(double c) {
    Eigen::Vector3d euler = angles();
    data.linear() = Euler(euler(0), euler(1), c).toRotationMatrix();
}

double Affine::q_w() const {
    return quaternion().w();
}

double Affine::q_x() const {
    return quaternion().x();
}

double Affine::q_y() const {
    return quaternion().y();
}

double Affine::q_z() const {
    return quaternion().z();
}

void Affine::set_quaternion(double w, double x, double y, double z) {
    data.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
}

Affine Affine::slerp(const Affine& affine, double t) const {
    Type result;
    Eigen::Quaterniond q_start(data.rotation());
    Eigen::Quaterniond q_end(affine.rotation());
    result.translation() = data.translation() + t * (affine.translation() - data.translation());
    result.linear() = q_start.slerp(t, q_end).toRotationMatrix();
    return Affine(result);
}

Affine Affine::getInnerRandom() const {
    std::random_device r;
    std::default_random_engine engine(r());

    Vector6d max = vector();
    Vector6d random;
    for (int i = 0; i < 6; i++) {
        std::uniform_real_distribution<double> distribution(-max(i), max(i));
        random(i) = distribution(engine);
    }

    return Affine(random);
}

std::string Affine::toString() const {
    Vector6d v = vector();
    return "[" + std::to_string(v(0)) + ", " + std::to_string(v(1)) + ", " + std::to_string(v(2))
      + ", " + std::to_string(v(3)) + ", " + std::to_string(v(4)) + ", " + std::to_string(v(5)) + "]";
}

} // namespace frankx
