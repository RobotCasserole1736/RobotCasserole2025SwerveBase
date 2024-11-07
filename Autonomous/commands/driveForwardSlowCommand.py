from AutoSequencerV2.command import Command
from drivetrain.drivetrainCommand import DrivetrainCommand
from wpilib import Timer

class DriveForwardSlowCommand(Command):
    def __init__(self):
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.returnDriveTrainCommand.velX = 0.25
        self.returnDriveTrainCommand.velY = 0.0
        self.returnDriveTrainCommand.velT = 0.0
        self.startTime = Timer.getFPGATimestamp()
        self.started = False
        pass

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.started = True

    def isRunning(self):
        return self.started and not self.isDone()

    def execute(self):
        return self.returnDriveTrainCommand

    def isDone(self):
        return Timer.getFPGATimestamp() - self.startTime >= 3

    def end(self,interrupt):
        pass
