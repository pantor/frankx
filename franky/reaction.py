from ._franky import Condition, CartesianPoseMotion, CartesianVelocityMotion, JointPositionMotion, \
    JointVelocityMotion, TorqueMotion, \
    CartesianPoseReaction as _CartesianPoseReaction, \
    CartesianVelocityReaction as _CartesianVelocityReaction, \
    JointPositionReaction as _JointPositionReaction, \
    JointVelocityReaction as _JointVelocityReaction, \
    TorqueReaction as _TorqueReaction

from .motion import Motion


class Reaction:
    _control_signal_type = None

    def __new__(cls, condition: Condition, motion: Motion):
        for reaction_type in _REACTION_TYPES:
            if isinstance(motion, reaction_type._motion_type):
                return reaction_type.__new__(reaction_type, condition, motion)
        raise TypeError(f"Unknown motion type {type(motion)}.")


class CartesianPoseReaction(_CartesianPoseReaction, Reaction):
    _motion_type = CartesianPoseMotion


class CartesianVelocityReaction(_CartesianVelocityReaction, Reaction):
    _motion_type = CartesianVelocityMotion


class JointPositionReaction(_JointPositionReaction, Reaction):
    _motion_type = JointPositionMotion


class JointVelocityReaction(_JointVelocityReaction, Reaction):
    _motion_type = JointVelocityMotion


class TorqueReaction(_TorqueReaction, Reaction):
    _motion_type = TorqueMotion


_REACTION_TYPES = [
    CartesianPoseReaction, CartesianVelocityReaction, JointPositionReaction, JointVelocityReaction, TorqueReaction]
