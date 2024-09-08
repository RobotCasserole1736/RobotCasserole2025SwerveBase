from drivetrain.drivetrainCommand import DrivetrainCommand
from wpimath.geometry import Pose2d
from utils.singleton import Singleton

class AutoDrive(metaclass=Singleton):
    def __init__(self):
        pass

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        # TODO - auto alignment strategies here
        return cmdIn # Default - no auto alignment