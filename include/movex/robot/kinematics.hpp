#pragma once

#include <affx/affine.hpp>


namespace movex {

template<size_t DoFs>
class KinematicChain {
    using Affine = affx::Affine;

    struct DenavitHartenbergParameter {
        double alpha; // [rad]
        double d, a; // [m]

        Affine get_transformation(double theta) const {
            Affine rot1 {0, 0, 0, theta, 0, 0};
            Affine trans1 {0, 0, d, 0, 0, 0};
            Affine trans2 {a, 0, 0, 0, 0, 0};
            Affine rot2 {0, 0, 0, 0, 0, alpha};
            return rot2 * trans2 * rot1 * trans1;
        }
    };

    std::array<DenavitHartenbergParameter, DoFs> parameters;
    Affine base;

public:
    explicit KinematicChain(const std::array<DenavitHartenbergParameter, DoFs>& parameters, const Affine& base): parameters(parameters), base(base) { }

    Affine forward_chain(const std::array<double, DoFs>& q) const {
        Affine result;
        for (size_t i = 0; i < DoFs; ++i) {
            result = result * parameters[i].get_transformation(q[i]);
        }
        return result * base;
    }
};

} // namespace movex
