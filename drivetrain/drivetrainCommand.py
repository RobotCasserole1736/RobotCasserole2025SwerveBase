from wpimath.geometry import Pose2d
from dataclasses import dataclass


# Represents desired drivetrain motion which is currently desired.
# Usually comes from a human driver, but could be from an autonomous momde or assist feature.
@dataclass
class DrivetrainCommand:
    velX:float = 0.0  # Field X velocity in meters/sec
    velY:float = 0.0  # Field Y velocity in meters/sec
    velT:float = 0.0  # Rotational speed in rad/sec
    desPose: Pose2d | None = None  # Current desired pose of the drivetrain, nor None if no pose is specified.
