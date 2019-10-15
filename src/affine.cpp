#include <frankx/affine.hpp>


namespace frankx {

Affine::Affine() {
    this->data = Eigen::Affine3d::Identity();
}

Affine::Affine(const Eigen::Affine3d& data) {
    this->data = data;
}

Affine::Affine(double x, double y, double z, double a, double b, double c) {
    ref_euler << a, b, c;
    data = Eigen::Translation<double, 3>(x, y, z) * Euler(a, b, c).toRotationMatrix();
}

Affine::Affine(const Vector6d& v): Affine(v(0), v(1), v(2), v(3), v(4), v(5)) { }

Affine::Affine(const Vector7d& v): Affine(v(0), v(1), v(2), v(3), v(4), v(5)) { }

Affine::Affine(const std::array<double, 16>& array) {
    Eigen::Affine3d affine(Eigen::Matrix4d::Map(array.data()));
    data = affine;
}

Affine::Affine(RMLVector<double> *rml_vector): Affine(rml_vector->VecData[0], rml_vector->VecData[1], rml_vector->VecData[2], rml_vector->VecData[3], rml_vector->VecData[4], rml_vector->VecData[5]) { }

Affine::Affine(const franka::CartesianPose& pose, bool offset) {
    Eigen::Affine3d affine(Eigen::Matrix4d::Map(pose.O_T_EE.data()));
    if (offset) {
        affine = affine.rotate(offset_euler);
    }
    data = affine;
}

Affine Affine::operator *(const Affine &a) const {
    Eigen::Affine3d result;
    result = data * a.data;
    return Affine(result);
}

Affine Affine::inverse() const {
    return Affine(data.inverse());
}

bool Affine::isApprox(const Affine &a) const {
    return data.isApprox(a.data);
}

Eigen::Ref<Eigen::Affine3d::MatrixType> Affine::matrix() {
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
    euler2 << euler[0] - M_PI, M_PI - euler[1], -M_PI + euler[2];

    if (euler2[1] > M_PI) {
        euler2[1] -= 2 * M_PI;
    }
    if (euler2[2] < -M_PI) {
        euler2[2] += 2 * M_PI;
    }

    if ((ref_euler - euler).norm() < (ref_euler - euler2).norm()) {
        return euler;
    }
    return euler2;
}

Eigen::Vector3d Affine::angles(const Eigen::Vector3d& new_ref_euler) {
    ref_euler = new_ref_euler;
    return angles();
}

Vector6d Affine::vector() const {
    Vector6d result;
    result << data.translation(), angles();
    return result;
}

Vector6d Affine::vector(const Eigen::Vector3d& new_ref_euler) {
    ref_euler = new_ref_euler;
    return vector();
}

Vector7d Affine::vector(double elbow) const {
    Vector7d result;
    result << data.translation(), angles(), elbow;
    return result;
}

Vector7d Affine::vector(double elbow, const Eigen::Vector3d& new_ref_euler) {
    ref_euler = new_ref_euler;
    return vector(elbow);
}

Vector7d Affine::vector(double elbow, const Vector7d& new_ref_vector) {
    ref_euler << new_ref_vector(3), new_ref_vector(4), new_ref_vector(5);
    return vector(elbow);
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

void Affine::rotate(const Eigen::Affine3d::LinearMatrixType &r) {
    data.rotate(r);
}

void Affine::prerotate(const Eigen::Affine3d::LinearMatrixType &r) {
    data.prerotate(r);
}

Eigen::Affine3d::LinearMatrixType Affine::rotation() const {
    Eigen::Affine3d::LinearMatrixType result;
    result << data.rotation();
    return result;
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
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(a, euler(1), euler(2)).toRotationMatrix();

    ref_euler(0) = a;
}

void Affine::set_b(double b) {
    Eigen::Vector3d euler = angles();
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(euler(0), b, euler(2)).toRotationMatrix();

    ref_euler(1) = b;
}

void Affine::set_c(double c) {
    Eigen::Vector3d euler = angles();
    data = Eigen::Translation<double, 3>(data.translation()) * Euler(euler(0), euler(1), c).toRotationMatrix();

    ref_euler(2) = c;
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
