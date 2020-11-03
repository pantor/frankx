#include <frankx/motion_impedance.hpp>


namespace frankx {

Affine ImpedanceMotion::getTarget() const {
    return target;
}

void ImpedanceMotion::setTarget(const Affine& new_target) {
    if (is_active) {
        target = new_target;
    }
    target_motion = ImpedanceMotion::TargetMotion::Exponential;
}

void ImpedanceMotion::setLinearRelativeTargetMotion(const Affine& relative_target, double duration) {
    linear_motion = {relative_target, duration};
    target_motion = ImpedanceMotion::TargetMotion::Linear;
}

void ImpedanceMotion::setSpiralTargetMotion(const Affine& center, double revolutions_per_second, double radius_per_revolution) {
    spiral_motion = {center, revolutions_per_second, radius_per_revolution};
    target_motion = ImpedanceMotion::TargetMotion::Spiral;
}

void ImpedanceMotion::addForceConstraint(Axis axis, double value) {
    if (is_active) {
        return;
    }

    force_constraints[axis] = value;
}

void ImpedanceMotion::addForceConstraint(std::optional<double> x, std::optional<double> y, std::optional<double> z) {
    if (is_active) {
        return;
    }

    if (x) {
        force_constraints[Axis::X] = x.value();
    }
    if (y) {
        force_constraints[Axis::Y] = y.value();
    }
    if (z) {
        force_constraints[Axis::Z] = z.value();
    }
}

bool ImpedanceMotion::isActive() const {
    return is_active;
}

void ImpedanceMotion::finish() {
    should_finish = true;
}

bool ImpedanceMotion::move(Robot* robot, const Affine& frame, MotionData& data) {
    if (type == Type::Joint) {
        throw std::runtime_error("joint impedance is not implemented yet.");
    }

    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

    franka::Model model = robot->loadModel();
    franka::RobotState initial_state = robot->readOnce();

    Affine initial_affine = Affine(initial_state.O_T_EE);
    Eigen::Vector3d position_d(initial_affine.translation());
    Eigen::Quaterniond orientation_d(initial_affine.quaternion());

    double time {0.0}, motion_init_time {0.0};
    auto impedance_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
        time += period.toSec();

        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        if (time == 0.0) {
            target = Affine(transform);
            is_active = true;
        }

        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;

        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }

        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -transform.linear() * error.tail(3);

        Eigen::VectorXd wrench_cartesian(6), tau_task(7), tau_d(7);
        wrench_cartesian = -stiffness * error - damping * (jacobian * dq);

        // Force constraints
        for (const auto force_constraint : force_constraints) {
            switch (force_constraint.first) {
                case Axis::X: {
                    wrench_cartesian(0) = force_constraint.second;
                } break;
                case Axis::Y: {
                    wrench_cartesian(1) = force_constraint.second;
                } break;
                case Axis::Z: {
                    // std::cout << robot_state.O_F_ext_hat_K[2] << std::endl;
                    wrench_cartesian(2) = force_constraint.second;
                } break;
            }
        }

        tau_task << jacobian.transpose() * wrench_cartesian;
        tau_d << tau_task + coriolis;

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

#ifdef WITH_PYTHON
        if (PyErr_CheckSignals() == -1) {
            is_active = false;
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }
#endif

        if (should_finish) {
            is_active = false;
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }

        // Update target with target motion
        switch (target_motion) {
            case ImpedanceMotion::TargetMotion::Exponential: {
                position_d = exponential_decay * target.translation() + (1.0 - exponential_decay) * position_d;
                orientation_d = orientation_d.slerp(exponential_decay, target.quaternion());
            } break;
            case ImpedanceMotion::TargetMotion::Linear: {
                if (!linear_motion.initialized) {
                    initial_affine = Affine(Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())));
                    motion_init_time = time;
                    linear_motion.initialized = true;
                }

                double transition_parameter = (time - motion_init_time) / (linear_motion.duration);
                if (transition_parameter <= 1.0) {  // [ms] to [s]
                    target = initial_affine.slerp(linear_motion.relative_target * initial_affine, transition_parameter);
                    position_d = target.translation();
                    orientation_d = target.quaternion();
                } else if (linear_motion.finish_after && transition_parameter > linear_motion.finish_wait_factor) { // Wait a bit longer to stop
                    should_finish = true;
                }
            } break;
            case ImpedanceMotion::TargetMotion::Spiral: {
                if (!spiral_motion.initialized) {
                    motion_init_time = time;
                    spiral_motion.initialized = true;
                }

                double time_diff = motion_init_time - time;
                // target = spiral_motion.center * Affine(0, 0, 0, 2 * pi * spiral_motion.revolutions_per_second * time) * Affine(spiral_motion.radius_increment_per_revolution * time);
                // position_d = target.translation();
                // orientation_d = target.quaternion();
            } break;
        }

        return franka::Torques(tau_d_array);
    };

    try {
        robot->control(impedance_callback);
        is_active = false;

    } catch (const franka::Exception& exception) {
        std::cout << exception.what() << std::endl;
        is_active = false;
        return false;
    }
    return true;
}

/* setCartesianImpedance({{motion.translational_stiffness, motion.translational_stiffness, motion.translational_stiffness, motion.rotational_stiffness, motion.rotational_stiffness, motion.rotational_stiffness}});

    double time = 0.0;
    auto motion_generator = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        if (time == 0.0) {
            franka::CartesianPose initial_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
            motion.target = Affine(initial_pose.O_T_EE);
            motion.is_active = true;
        }

#ifdef WITH_PYTHON
        if (PyErr_CheckSignals() == -1) {
            motion.is_active = false;
            return franka::CartesianPose(motion.target.array());
        }
#endif

        if (motion.should_finish) {
            motion.is_active = false;
            return franka::MotionFinished(motion.target.array());
        }

        return franka::CartesianPose(motion.target.array());
    };

    try {
        control(motion_generator, franka::ControllerMode::kCartesianImpedance);

    } catch (const franka::Exception& exception) {
        std::cout << exception.what() << std::endl;
        return false;
    }
    return true; */

} // namespace frankx
