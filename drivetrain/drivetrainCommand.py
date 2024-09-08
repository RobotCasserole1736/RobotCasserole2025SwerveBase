from wpimath.geometry import Pose2d


# Represents desired drivetrain motion which is currently desired.
# Usually comes from a human driver, but could be from an autonomous momde or assist feature.
class DrivetrainCommand:
    def __init__(self):
        self.velX = 0.0  # Field X velocity in meters/sec
        self.velY = 0.0  # Field Y velocity in meters/sec
        self.velT = 0.0  # Rotational speed in rad/sec
        self.desPose: Pose2d | None = None  # Current desired pose of the drivetrain, nor None if no pose is specified.
