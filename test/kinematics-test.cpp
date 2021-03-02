#include <iostream>
#include <chrono>

#include <Eigen/Core>

#include <frankx/frankx.hpp>


using namespace frankx;


int main(int argc, char *argv[]) {
    std::array<double, 7> q = {-1.4554923355, 1.1540154275, 1.50061583024, -2.30909621308, -1.3141626213, 1.93919787437, 0.028150367940};
    std::array<double, 6> x = {0.473971, -0.307686, 0.340767, 0.545131, -0.510650, -0.552355};

    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix<double, 6, 1> x_target = Eigen::Map<Eigen::Matrix<double, 6, 1>>(x.data(), x.size());
    Eigen::Matrix<double, 7, 1> q_current = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q.data(), q.size());

    auto q_new = Kinematics::inverse(x_target, q_current, Kinematics::NullSpaceHandling {1, 1.0});

    auto stop = std::chrono::high_resolution_clock::now();
    double t = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count() / 1000.0;

    std::cout << "q_new: " << q_new << std::endl;
    std::cout << "calculation time: " << t << std::endl;
}
