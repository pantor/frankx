#pragma once

namespace franky {

enum ControlSignalType {
  Torques,
  JointVelocities,
  JointPositions,
  CartesianVelocities,
  CartesianPose
};

}  // namespace franky
