from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from utils.singleton import Singleton
from navigation.dynamicPathPlanner import GOAL_PICKUP, GOAL_SPEAKER, DynamicPathPlanner

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

    def getTrajectory(self) -> Trajectory|None:
        return self._dpp.curTraj
    
    def getWaypoints(self) -> list[Pose2d]:
        retArr = []

        if(self._dpp.waypoints is not None):
            retArr.append(self._dpp.curPose)
            for trans in self._dpp.waypoints:
                retArr.append(Pose2d(trans, Rotation2d.fromDegrees(0)))
            retArr.append(self._dpp.curGoal.endPose)

        return retArr

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
            retCmd = self._dpp.get()

        self._toSpeakerPrev = self._toSpeaker
        self._toPickupPrev = self._toPickup

        return retCmd