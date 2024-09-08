import wpilib

from AutoSequencerV2.command import Command


class WaitCommand(Command):
    def __init__(self, waitDur):
        self.waitDur = waitDur
        self.endTime = 0

    def initialize(self):
        self.endTime = wpilib.Timer.getFPGATimestamp() + self.waitDur

    def isDone(self):
        return wpilib.Timer.getFPGATimestamp() >= self.endTime

    def getName(self):
        return f"Wait {self.waitDur}s"
