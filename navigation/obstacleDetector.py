
from wrappers.wrapperedObstaclePhotonCamera import WrapperedObstaclePhotonCamera
from drivetrain.drivetrainPhysical import ROBOT_TO_FRONT_CAM
from wpimath.geometry import Pose2d
from navigation.obstacles import Obstacle, PointObstacle

class ObstacleDetector():
    def __init__(self):
        self.frontCam = WrapperedObstaclePhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM)

    def getObstacles(self, curPose:Pose2d) -> list['Obstacle']:
        retList = []

        self.frontCam.update()
        for obs in self.frontCam.getObstacles():
            obsPose = curPose.translation() + obs
            retList.append(PointObstacle(location=obsPose, strength=0.7))
        
        # TODO - add other cameras and their observations

        return retList

