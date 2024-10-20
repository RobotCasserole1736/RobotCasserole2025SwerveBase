from wpimath.geometry import Pose2d, Rotation2d

"""
Constants related to navigation
"""


# Happy Constants for the goal poses we may want to drive to
GOAL_PICKUP = Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0))
GOAL_SPEAKER = Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(180.0))
