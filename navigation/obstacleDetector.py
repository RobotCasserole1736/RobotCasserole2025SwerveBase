
from wrappers.wrapperedObstaclePhotonCamera import WrapperedObstaclePhotonCamera
from drivetrain.drivetrainPhysical import ROBOT_TO_FRONT_CAM
from wpimath.geometry import Pose2d
from navigation.forceGenerators import ForceGenerator, PointObstacle

class ObstacleDetector():
    """
    Class to use a PhotonVision-equipped coprocoessor and camera to deduce the location of obstacles on the field and report them
    """
    def __init__(self):
        self.frontCam = WrapperedObstaclePhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM)

    def getObstacles(self, curPose:Pose2d) -> list['ForceGenerator']:
        """
        Returns the currently observed obstacles
        """
        retList = []

        self.frontCam.update()
        for obs in self.frontCam.getObstacles():
            obsPose = curPose.translation() + obs
            retList.append(PointObstacle(location=obsPose, strength=0.7))
        
        # TODO - add other cameras and their observations

        return retList

