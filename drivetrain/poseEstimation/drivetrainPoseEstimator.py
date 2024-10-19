import random

from wpilib import ADIS16470_IMU
import wpilib
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Twist2d
from drivetrain.drivetrainPhysical import (
    kinematics,
    ROBOT_TO_LEFT_CAM,
    ROBOT_TO_RIGHT_CAM,
)
from drivetrain.poseEstimation.drivetrainPoseTelemetry import DrivetrainPoseTelemetry
from utils.faults import Fault
from utils.signalLogging import addLog
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera


class DrivetrainPoseEstimator:
    """Wrapper class for all sensors and logic responsible for estimating where the robot is on the field"""

    def __init__(self, initialModuleStates):
        self.curEstPose = Pose2d()
        self.curDesPose = Pose2d()
        self.gyro = ADIS16470_IMU()
        self.gyroDisconFault = Fault("Gyro Disconnected")

        self.cams = [
            WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
            WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
        ]
        self.camTargetsVisible = False

        self.poseEst = SwerveDrive4PoseEstimator(
            kinematics, self._getGyroAngle(), initialModuleStates, self.curEstPose
        )
        self.lastModulePositions = initialModuleStates
        self.curRawGyroAngle = Rotation2d()
        self.telemetry = DrivetrainPoseTelemetry()

        self._simPose = Pose2d()

        self.useAprilTags = True

        addLog("PE Vision Targets Seen", lambda: self.camTargetsVisible, "bool")
        addLog("PE Gyro Angle", self.curRawGyroAngle.degrees, "deg")

    def setKnownPose(self, knownPose):
        """Reset the robot's estimated pose to some specific position. This is useful if we know with certanty
        we are at some specific spot (Ex: start of autonomous)

        Args:
            knownPose (Pose2d): The pose we know we're at
        """
        if wpilib.TimedRobot.isSimulation():
            self._simPose = knownPose
            self.curRawGyroAngle = knownPose.rotation()

        self.poseEst.resetPosition(
            self.curRawGyroAngle, self.lastModulePositions, knownPose
        )

    def update(self, curModulePositions, curModuleSpeeds):
        """Periodic update, call this every 20ms.

        Args:
            curModulePositions (list[SwerveModuleState]): current module angle
            and wheel positions as read in from swerve module sensors
        """

        # Add any vision observations to the pose estimate
        self.camTargetsVisible = False

        if(self.useAprilTags):
            for cam in self.cams:
                cam.update(self.curEstPose)
                observations = cam.getPoseEstimates()
                for observation in observations:
                    self.poseEst.addVisionMeasurement(
                        observation.estFieldPose, observation.time
                    )
                    self.camTargetsVisible = True
                self.telemetry.addVisionObservations(observations)


        # Read the gyro angle
        self.gyroDisconFault.set(not self.gyro.isConnected())
        if wpilib.TimedRobot.isSimulation():
            # Simulate an angle based on (simulated) motor speeds with some noise
            chSpds = kinematics.toChassisSpeeds(curModuleSpeeds)
            self._simPose = self._simPose.exp(
                Twist2d(chSpds.vx * 0.02, chSpds.vy * 0.02, chSpds.omega * 0.02)
            )
            noise = Rotation2d.fromDegrees(random.uniform(-0.0, 0.0))
            self.curRawGyroAngle = self._simPose.rotation() + noise
        else:
            # Use real hardware
            self.curRawGyroAngle = self._getGyroAngle()

        # Update the WPILib Pose Estimate
        self.poseEst.update(self.curRawGyroAngle, curModulePositions)
        self.curEstPose = self.poseEst.getEstimatedPosition()

        # Record the estimate to telemetry/logging-
        self.telemetry.update(self.curEstPose, [x.angle for x in curModulePositions])

        # Remember the module positions for next loop
        self.lastModulePositions = curModulePositions

    def getCurEstPose(self):
        """
        Returns:
            Pose2d: The most recent estimate of where the robot is at
        """
        return self.curEstPose
    
    def setUseAprilTags(self, use):
        self.useAprilTags = use

    # Local helper to wrap the real hardware angle into a Rotation2d
    def _getGyroAngle(self):
        return Rotation2d().fromDegrees(self.gyro.getAngle(self.gyro.getYawAxis()))