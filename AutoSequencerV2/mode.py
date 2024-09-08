from wpimath.geometry import Pose2d
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup


# An auto mode is something our robot might do during autonomous
# THe drive team selects the mode before the match
# The robot executes the mode during autonomous
# Modes must have a human-readable name, return a group of commands, and an
# initial drivetrain pose (IE, where is it expected the drive team placed the
# robot for this autonomous routine?)
class Mode:
    def __init__(self, name=None):
        if name:
            self._name = name
        else:
            self._name = self.__class__.__name__

    def getCmdGroup(self):
        return SequentialCommandGroup([])

    def getInitialDrivetrainPose(self):
        return Pose2d(0, 0, 0)

    def getName(self):
        return self._name
