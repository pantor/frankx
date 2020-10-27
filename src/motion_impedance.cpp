#include <frankx/motion_impedance.hpp>


namespace frankx {

Affine ImpedanceMotion::getTarget() const {
    return target;
}

void ImpedanceMotion::setTarget(const Affine& new_target) {
    if (is_active) {
        target = new_target;
    }
}

bool ImpedanceMotion::isActive() const {
    return is_active;
}

void ImpedanceMotion::finish() {
    should_finish = true;
}

} // namespace frankx
