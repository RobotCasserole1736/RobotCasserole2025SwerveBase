from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import Trajectory
from drivetrain.controlStrategies.holonomicDriveController import HolonomicDriveController
from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.obstacleDetector import ObstacleDetector
from utils.signalLogging import log
from utils.singleton import Singleton
from navigation.repulsorFieldPlanner import GOAL_PICKUP, GOAL_SPEAKER, RepulsorFieldPlanner
from drivetrain.drivetrainPhysical import MAX_DT_LINEAR_SPEED
from utils.allianceTransformUtils import transform



SPEED_SCALAR = 0.75

class AutoDrive(metaclass=Singleton):
    def __init__(self):
        self._toSpeaker = False
        self._toPickup = False
        self.rfp = RepulsorFieldPlanner()
        self._trajCtrl = HolonomicDriveController()
        self._telemTraj = []
        self._obsDet = ObstacleDetector()
        self._prevCmd:DrivetrainCommand|None = None

    def setRequest(self, toSpeaker, toPickup) -> None:
        self._toSpeaker = toSpeaker
        self._toPickup = toPickup

    def getTrajectory(self) -> Trajectory|None:
        return None # TODO
    
    def updateTelemetry(self) -> None:        
        self._telemTraj = self.rfp.updateTelemetry()

    def getWaypoints(self) -> list[Pose2d]:
        return self._telemTraj
    
    def getObstacles(self) -> list[Translation2d]:
        return self.rfp.getObstacleTransList()
    
    def isRunning(self)->bool:
        return self.rfp.goal != None

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:

        retCmd = cmdIn # default - no auto driving

        for obs in self._obsDet.getObstacles(curPose):
            self.rfp.add_obstcale_observaton(obs)

        self.rfp._decay_observations()

        # Handle command changes
        if(self._toPickup):
            self.rfp.setGoal(transform(GOAL_PICKUP))
        elif(self._toSpeaker):
            self.rfp.setGoal(transform(GOAL_SPEAKER))
        elif(not self._toSpeaker and not self._toPickup):
            self.rfp.setGoal(None)

        # If being asked to auto-align, use the command from the dynamic path planner
        if(self._toPickup or self._toSpeaker):

            if(self._prevCmd is None or self._prevCmd.desPose is None):
                olCmd = self.rfp.update(curPose, MAX_DT_LINEAR_SPEED*0.02*SPEED_SCALAR)
            else:
                olCmd = self.rfp.update(self._prevCmd.desPose, MAX_DT_LINEAR_SPEED*0.02*SPEED_SCALAR)

            log("AutoDrive FwdRev Cmd", olCmd.velX, "mps")
            log("AutoDrive Strafe Cmd", olCmd.velY, "mps")
            log("AutoDrive Rot Cmd", olCmd.velT, "radpers")

            if( olCmd.desPose is not None):
                retCmd = self._trajCtrl.update2(olCmd.velX, olCmd.velY, olCmd.velT, olCmd.desPose, curPose)
            
            self._prevCmd = retCmd
        else:
            self._prevCmd = None


        return retCmd