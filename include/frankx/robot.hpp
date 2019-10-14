#include <memory>

#include <franka/exception.h>
#include <franka/robot.h>


namespace frankx {
  class Robot: public franka::Robot {
    const double max_translation_velocity {1.7}; // [m/s]
    const double max_rotation_velocity {2.5}; // [rad/s]
    const double max_translation_acceleration {13.0}; // [m/s²]
    const double max_rotation_acceleration {25.0}; // [rad/s²]
    const double max_translation_jerk {6500.0}; // [m/s³]
    const double max_rotation_jerk {12500.0}; // [rad/s³]

    double velocity_rel {1.0};
    double acceleration_rel {1.0};
    double jerk_rel {1.0};


  public:
    /**
     * Connects to a robot at the given FCI IP address.
     */
    Robot(std::string fci_ip): franka::Robot(fci_ip) {}

    void setDefault() {
      setCollisionBehavior(
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 10.0}},
          {{20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 10.0}},
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
          {{20.0, 20.0, 20.0, 10.0, 10.0, 10.0}},
          {{20.0, 20.0, 20.0, 10.0, 10.0, 10.0}}
      );
      setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    }


    void move(const WaypointMotion& motion) {
      constexpr int degrees_of_freedoms {7};
      constexpr double control_rate {0.001};

      RMLPositionFlags flags;
      int result_value = 0;

      const auto rml = std::make_unique<ReflexxesAPI>(degrees_of_freedoms, control_rate);
      auto input_parameters = std::make_unique<RMLPositionInputParameters>(degrees_of_freedoms);
      auto output_parameters = std::make_unique<RMLPositionOutputParameters>(degrees_of_freedoms);

      setVector(input_parameters->SelectionVector, VectorCartRotElbow(true, true, true));
      setVector(input_parameters->MaxVelocityVector, VectorCartRotElbow(0.3, 0.3, 0.3));
      setVector(input_parameters->MaxAccelerationVector, VectorCartRotElbow(1.0, 1.0, 1.0));
      setVector(input_parameters->MaxJerkVector, VectorCartRotElbow(5.0, 5.0, 5.0));


      auto waypoint_iterator = motion.waypoints.begin();
      Vector7d old_vector = Vector7d::Zero();

      double time = 0.0;
      control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        if (time == 0.0) {
          franka::CartesianPose initial_pose = franka::CartesianPose(robot_state.O_T_EE_c, robot_state.elbow_c);
          std::array<double, 7> initial_velocity = {robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2], robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5], robot_state.delbow_c[0]};

          Vector7d initial_vector = Vector(initial_pose, old_vector);
          old_vector = initial_vector;
          setVector(input_parameters->CurrentPositionVector, initial_vector);
          setVector(input_parameters->CurrentVelocityVector, initial_velocity);
          setZero(input_parameters->CurrentAccelerationVector);

          Waypoint current_waypoint = *waypoint_iterator;
          setVector(input_parameters->TargetPositionVector, Vector(current_waypoint.getTargetPose(), old_vector));
          setVector(input_parameters->TargetVelocityVector, current_waypoint.getTargetVelocity());
        }

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
          result_value = rml->RMLPosition(*input_parameters, output_parameters.get(), flags);

          if (result_value == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
            waypoint_iterator += 1;

            if (waypoint_iterator == motion.waypoints.end()) {
              return franka::MotionFinished(getCartesianPose(input_parameters->CurrentPositionVector));
            }

            Waypoint current_waypoint = *waypoint_iterator;
            setVector(input_parameters->TargetPositionVector, Vector(current_waypoint.getTargetPose(), old_vector));
            setVector(input_parameters->TargetVelocityVector, current_waypoint.getTargetVelocity());
          }

          *input_parameters->CurrentPositionVector = *output_parameters->NewPositionVector;
          *input_parameters->CurrentVelocityVector = *output_parameters->NewVelocityVector;
          *input_parameters->CurrentAccelerationVector = *output_parameters->NewAccelerationVector;
        }

        return getCartesianPose(output_parameters->NewPositionVector);
      }, franka::ControllerMode::kCartesianImpedance);
    }
  };
}
