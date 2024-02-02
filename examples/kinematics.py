from franky import Affine, Kinematics, NullSpaceHandling

if __name__ == "__main__":
    # Some initial joint configuration
    q = [-1.45549, 1.15401, 1.50061, -2.30909, -1.3141, 1.9391, 0.02815]

    # Calculate the forward kinematics
    x = Kinematics.forward(q)
    print(f"Effector position: {x}")

    # Define new target position
    x_new = Affine([0.1, 0.0, 0.0]) * x

    # Franka has 7 DoFs, so what to do with the remaining Null space?
    null_space = NullSpaceHandling(2, 1.4)  # Set elbow joint to 1.4

    # Inverse kinematic with target, initial joint angles, and Null space configuration
    q_new = Kinematics.inverse(x_new.vector(), q, null_space)

    print(f"New position: {x_new}")
    print(f"New joints: {q_new}")
