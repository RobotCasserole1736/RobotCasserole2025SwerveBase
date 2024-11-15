from dataclasses import dataclass
import wpilib
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonCamera import setVersionCheckEnabled
from utils.fieldTagLayout import FieldTagLayout
from utils.faults import Fault

# Describes one on-field pose estimate from the a camera at a specific time.
@dataclass
class CameraPoseObservation:
    time:float
    estFieldPose:Pose2d
    xyStdDev:float=1.0 # std dev of error in measurment, units of meters.
    rotStdDev:float=degreesToRadians(50.0) # std dev of measurement, in units of radians

# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPoseEstPhotonCamera:
    def __init__(self, camName, robotToCam):
        setVersionCheckEnabled(False)

        self.cam = PhotonCamera(camName)

        self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates:list[CameraPoseObservation] = []
        self.robotToCam = robotToCam

    def update(self, prevEstPose:Pose2d):

        self.poseEstimates = []

        if(not self.cam.isConnected()):
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return

        # Grab whatever the camera last reported for observations in a camera frame
        # Note: Results simply report "I processed a frame". There may be 0 or more targets seen in a frame
        res = self.cam.getLatestResult()

        # hack - results also have a more accurate "getTimestamp()", but this is
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        # TODO: in 2025, fix this to actually use the real latency
        latency = 0.05 # a total guess
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
            tgtID = target.getFiducialId()
            if tgtID >= 0:
                # Only handle valid ID's
                tagFieldPose = FieldTagLayout().lookup(tgtID)
                if tagFieldPose is not None:
                    # Only handle known tags
                    poseCandidates:list[Pose2d] = []
                    if target.getPoseAmbiguity() <= .5:
                        poseCandidates.append(
                            self._toFieldPose(tagFieldPose, target.getBestCameraToTarget())
                        )
                        poseCandidates.append(
                            self._toFieldPose(tagFieldPose, target.getAlternateCameraToTarget())
                    )

                    # Filter candidates in this frame to only the valid ones
                    filteredCandidates:list[Pose2d] = []
                    for candidate in poseCandidates:
                        onField = self._poseIsOnField(candidate)
                        # Add other filter conditions here
                        if onField and target.getBestCameraToTarget().translation().norm() <= 4:
                            filteredCandidates.append(candidate)

                    # Pick the candidate closest to the last estimate
                    bestCandidate:(Pose2d|None) = None
                    bestCandidateDist = 99999999.0
                    for candidate in filteredCandidates:
                        delta = (candidate - prevEstPose).translation().norm()
                        if delta < bestCandidateDist:
                            # This candidate is better, use it
                            bestCandidate = candidate
                            bestCandidateDist = delta

                    # Finally, add our best candidate the list of pose observations
                    if bestCandidate is not None:
                        # TODO: we can probably get better standard deviations than just
                        # assuming the default. Check out what 254 did in 2024:
                        # https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java#L381
                        self.poseEstimates.append(
                            CameraPoseObservation(obsTime, bestCandidate)
                        )

    def getPoseEstimates(self):
        return self.poseEstimates

    def _toFieldPose(self, tgtPose, camToTarget):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = 0.0 <= y <= feetToMeters(27.0)
        inX = 0.0 <= x <= feetToMeters(54.0)
        return inX and inY
