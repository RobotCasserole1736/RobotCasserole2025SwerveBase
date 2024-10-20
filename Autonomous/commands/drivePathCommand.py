import os
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from jormungandr import choreo
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform


class DrivePathCommand(Command):
    def __init__(self, pathFile):
        self.name = pathFile

        self.trajCtrl = Trajectory()

        # Get the internal path file
        absPath = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "deploy",
                "choreo",
                pathFile + ".traj",
            )
        )

        self.path = choreo.fromFile(absPath)
        self.done = False
        self.startTime = (
            -1
        )
       
        # we'll populate these for real later, just declare they'll exist
        self.duration = self.path.getTotalTime()
        self.drivetrain = DrivetrainControl()
        self.poseTelem = self.drivetrain.poseEst._telemetry

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.poseTelem.setChoreoTrajectory(self.path)

    def execute(self):
        curTime = Timer.getFPGATimestamp() - self.startTime
        curState = self.path.sample(curTime)

        curState = transform(curState)

        self.trajCtrl.setCmd(curState)

        if curTime >= self.duration:
            self.trajCtrl.setCmd(None)
            self.poseTelem.setChoreoTrajectory(None)
            self.done = True

    def isDone(self):
        return self.done

    def end(self,interrupt):
        self.trajCtrl.setCmd(None)
        self.poseTelem.setChoreoTrajectory(None)

    def getName(self):
        return f"Drive Trajectory {self.name}"