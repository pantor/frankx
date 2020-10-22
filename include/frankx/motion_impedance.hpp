#pragma once
// Inpspired by: https://github.com/frankaemika/libfranka/blob/develop/examples/cartesian_impedance_control.cpp

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <frankx/affine.hpp>
#include <frankx/utils.hpp>


namespace frankx {

class ImpedanceMotion {
    bool is_active {false};

public:
    const double translational_stiffness {150.0};
    const double rotational_stiffness {10.0};

    Affine equilibrium;

    ImpedanceMotion() { }

    void setEquilibrium(Affine new_equilibrium) {
        equilibrium = new_equilibrium;
    }

    bool isActive() {
        return is_active;
    }
};

} // namespace frankx