from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.holonomicDriveController import (
    HolonomicDriveController,
)
from jormungandr.choreo import ChoreoTrajectoryState
from utils.singleton import Singleton


class Trajectory(metaclass=Singleton):
    def __init__(self):
        self.trajCtrl = HolonomicDriveController()
        self.curTrajCmd = None

    def setCmd(self, cmd: ChoreoTrajectoryState | None):
        """Send commands to the robot for motion as a part of following a trajectory

        Args:
            cmd (PathPlannerState): PathPlanner trajectory sample for the current time, or None for inactive.
        """
        self.curTrajCmd = cmd

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.curTrajCmd is not None:
            return self.trajCtrl.update(self.curTrajCmd, curPose)
        else:
            return cmdIn
