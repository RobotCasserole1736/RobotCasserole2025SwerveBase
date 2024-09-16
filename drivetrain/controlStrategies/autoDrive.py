from drivetrain.drivetrainCommand import DrivetrainCommand
from wpimath.geometry import Pose2d
from utils.singleton import Singleton
from navigation.dynamicPathPlanner import DynamicPathPlanner, GOAL_PICKUP, GOAL_SPEAKER

class AutoDrive(metaclass=Singleton):
    def __init__(self):
        self._toSpeaker = False
        self._toPickup = False
        self._toSpeakerPrev = False
        self._toPickupPrev = False
        self._dpp = DynamicPathPlanner()


    def setRequest(self, toSpeaker, toPickup) -> None:
        self._toSpeaker = toSpeaker
        self._toPickup = toPickup

    def getTrajectory(self):
        return self._dpp.curTraj

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:

        retCmd = cmdIn # default - no auto driving

        # Handle command changes
        if(self._toPickup and not self._toPickupPrev):
            # Just started going to the pickup, change the goal
            self._dpp.changeGoal(GOAL_PICKUP, curPose)
        elif(self._toSpeaker and not self._toSpeakerPrev):
            # Just started going to the speaker, change the goal
            self._dpp.changeGoal(GOAL_SPEAKER, curPose)

        # If being asked to auto-align, use the command from the dynamic path planner
        if(self._toPickup or self._toSpeaker):
            cmdIn = self._dpp.get()

        self._toSpeakerPrev = self._toSpeaker
        self._toPickupPrev = self._toPickup

        return retCmd