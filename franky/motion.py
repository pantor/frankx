from typing import Union

from ._franky import CartesianPoseMotion, CartesianVelocityMotion, JointPositionMotion, JointVelocityMotion, \
    TorqueMotion

Motion = Union[CartesianPoseMotion, CartesianVelocityMotion, JointPositionMotion, JointVelocityMotion, TorqueMotion]
