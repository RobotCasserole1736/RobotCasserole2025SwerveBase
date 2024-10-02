import wpilib
from wpimath.geometry import Pose2d, Transform3d, Rotation2d, Translation2d
from wpimath.units import degreesToRadians
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonCamera import setVersionCheckEnabled
from utils.faults import Fault
import math


class CameraObstacleObservation:
    def __init__(self, time:float, estFieldPose:Pose2d, trustworthiness=1.0):
        self.time = time
        self.estFieldPose = estFieldPose
        self.trustworthiness = trustworthiness  # TODO - not used yet

MIN_AREA=10.0 #idk tune this if we are reacting to small targets

def calculateDistanceToTargetMeters(
        cameraHeightMeters:float,
        targetHeightMeters:float,
        cameraPitchRadians:float,
        targetPitchRadians:float):
    return (targetHeightMeters - cameraHeightMeters) / math.tan(cameraPitchRadians + targetPitchRadians)


def estimateCameraToTargetTranslation(targetDistanceMeters:float, yaw:Rotation2d):
        return Translation2d(
                yaw.cos() * targetDistanceMeters, yaw.sin() * targetDistanceMeters)

# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedObstaclePhotonCamera:
    def __init__(self, camName, robotToCam:Transform3d):
        setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)

        self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.obstacleEstimates:list[Translation2d] = []
        self.robotToCam = robotToCam

    def update(self):

        self.obstacleEstimates = []

        if(not self.cam.isConnected()):
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return

        # Grab whatever the camera last reported for observations in a camera frame
        # Note: Results simply report "I processed a frame". There may be 0 or more targets seen in a frame
        res = self.cam.getLatestResult()

        # MiniHack - results also have a more accurate "getTimestamp()", but this is
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        latency = res.getLatencyMillis()
        obsTime = wpilib.Timer.getFPGATimestamp() - latency
        
        # Update our disconnected fault since we have something from the camera
        self.disconFault.setNoFault()

        # Process each target.
        # Each target has multiple solutions for where you could have been at on the field
        # when you observed it
        # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
        # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
        # We want to select the best possible pose per target
        # We should also filter out targets that are too far away, and poses which
        # don't make sense.
        for target in res.getTargets():
            # Transform both poses to on-field poses
            if (target.getArea()>MIN_AREA):
                # Use algorithm described at 
                # https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
                # to estimate distance from the camera to the target.
                dist = calculateDistanceToTargetMeters(
                    self.robotToCam.translation().Z(),
                    0.05, # Assume the average bumper starts 5cm off the ground
                    self.robotToCam.rotation().Y(), # Pitch is rotation about the Y axis
                    degreesToRadians(target.getPitch())
                )
                camToObstacle = estimateCameraToTargetTranslation(
                    dist,
                    Rotation2d.fromDegrees(target.getYaw())
                )
                
                self.obstacleEstimates.append(camToObstacle)

    def getObstacles(self) -> list[Translation2d]:
        return self.obstacleEstimates
