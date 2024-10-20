from typing import overload
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from jormungandr.choreoTrajectory import ChoreoTrajectoryState
from utils.constants import FIELD_X_M

"""
 Utilities to help transform from blue alliance to red if needed
 We chose a coordinate system where the origin is always in the 
 bottom left on the blue alliance.

 Note that this assumes the "mirroring" that happened in 2023 and 2024.
 A diagonally symmetric field (like 2020) would not need this.
"""


# Simple utility to check if we're on the red alliance (and therefor doing transformation is needed)
def onRed():
    return (
        wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    )

# Base transform for X axis to flip to the other side of the field.
def transformX(xIn):
    if onRed():
        return FIELD_X_M - xIn
    else:
        return xIn


# Note that Y axis does not need any flipping

# Other types are flipped in the transform function


# The following typehints remove vscode errors by telling the linter
# exactly how the transform() function handles different types
@overload
def transform(valIn: Rotation2d) -> Rotation2d:
    pass


@overload
def transform(valIn: Translation2d) -> Translation2d:
    pass


@overload
def transform(valIn: Pose2d) -> Pose2d:
    pass


@overload
def transform(valIn: ChoreoTrajectoryState) -> ChoreoTrajectoryState:
    pass


# Actual implementation of the transform function
def transform(valIn):
    if isinstance(valIn, Rotation2d):
        if onRed():
            return Rotation2d.fromDegrees(180) - valIn
        else:
            return valIn

    elif isinstance(valIn, Translation2d):
        if onRed():
            return Translation2d(transformX(valIn.X()), valIn.Y())
        else:
            return valIn

    elif isinstance(valIn, Transform2d):
        if onRed():
            trans = transform(valIn.translation())
            rot = transform(valIn.rotation())
            return Transform2d(trans, rot)
        else:
            return valIn

    elif isinstance(valIn, Pose2d):
        if onRed():
            trans = transform(valIn.translation())
            rot = transform(valIn.rotation())
            return Pose2d(trans, rot)
        else:
            return valIn

    elif isinstance(valIn, ChoreoTrajectoryState):
        if onRed():
            return valIn.flipped()
        else:
            return valIn

    else:
        raise TypeError("transform function received unknown type")
