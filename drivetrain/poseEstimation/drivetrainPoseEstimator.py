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
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

# Convienent abreviations for the types that we'll be passing around here.
# This is primarily driven by wpilib's conventions:
# 1) Swerve objects will be put into a tuple, with the length of the tuple equal to the number of modules
# 2) "State" refers to the speed of the wheel, plus the position of the azimuth
# 3) "Position" refers to the position of the wheel, plus the position of the azimuth
# Wheel position is preferred for odometry, since errors in velocity don't accumulate over time
# This is especially important with Rev motors which filter their velocity by a huge amount
# but the position is fairly accurate. 
posTuple_t = tuple[SwerveModulePosition,SwerveModulePosition,SwerveModulePosition,SwerveModulePosition]
stateTuple_t = tuple[SwerveModuleState,SwerveModuleState,SwerveModuleState,SwerveModuleState]

class DrivetrainPoseEstimator:
    """Wrapper class for all sensors and logic responsible for estimating where the robot is on the field"""

    def __init__(self, initialModulePositions:posTuple_t):

        # Represents our current best-guess as to our location on the field.
        self._curEstPose = Pose2d()

        # Gyroscope - measures our rotational velocity.
        # Fairly accurate and trustworthy, but not a full pose estimate
        self._gyro = ADIS16470_IMU()
        self._gyroDisconFault = Fault("Gyroscope not sending data")
        self._curRawGyroAngle = Rotation2d()

        # Cameras - measure our position on the field from apriltags
        # Generally accurate, but slow and laggy. Might need to be disabled
        # if the robot isn't flat on the ground for some reason.
        self.cams = [
            WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
            WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
        ]
        self._camTargetsVisible = False
        self._useAprilTags = True

        # The kalman filter to fuse all sources of information together into a single
        # best-estimate of the pose of the field
        self._poseEst = SwerveDrive4PoseEstimator(
            kinematics, self._getGyroAngle(), initialModulePositions, self._curEstPose
        )
        self._lastModulePositions = initialModulePositions

        # Logging and Telemetry
        addLog("PE Vision Targets Seen", lambda: self._camTargetsVisible, "bool")
        #addLog("PE Gyro Angle", self._curRawGyroAngle.degrees, "deg")
        self._telemetry = DrivetrainPoseTelemetry()

        # Simulation Only - maintain a rough estimate of pose from velocities
        # Using just inverse kinematics, no kalman filter. This is used only
        # to produce a reasonable-looking simulated gyroscope.
        self._simPose = Pose2d()

    def setKnownPose(self, knownPose:Pose2d):
        """Reset the robot's estimated pose to some specific position. This is useful if we know with certanty
        we are at some specific spot (Ex: start of autonomous)

        Args:
            knownPose (Pose2d): The pose we know we're at
        """
        if wpilib.TimedRobot.isSimulation():
            self._simPose = knownPose
            self._curRawGyroAngle = knownPose.rotation()

        self._poseEst.resetPosition(
            self._curRawGyroAngle, self._lastModulePositions, knownPose
        )

    def update(self, curModulePositions:posTuple_t, curModuleSpeeds:stateTuple_t):
        """Periodic update, call this every 20ms.

        Args:
            curModulePositions  current module angle
            and wheel positions as read in from swerve module sensors
        """

        # Add any vision observations to the pose estimate
        self._camTargetsVisible = False

        if(self._useAprilTags):
            for cam in self.cams:
                cam.update(self._curEstPose)
                observations = cam.getPoseEstimates()
                for observation in observations:
                    self._poseEst.addVisionMeasurement(
                        observation.estFieldPose, observation.time, (observation.xyStdDev, observation.xyStdDev, observation.rotStdDev)
                    )
                    self._camTargetsVisible = True
                self._telemetry.addVisionObservations(observations)


        # Read the gyro angle
        self._gyroDisconFault.set(not self._gyro.isConnected())
        if wpilib.TimedRobot.isSimulation():
            # Simulated Gyro
            # Simulate an angle based on (simulated) motor speeds with some noise
            chSpds = kinematics.toChassisSpeeds(curModuleSpeeds)
            self._simPose = self._simPose.exp(
                Twist2d(chSpds.vx * 0.02, chSpds.vy * 0.02, chSpds.omega * 0.02)
            )
            noise = Rotation2d.fromDegrees(random.uniform(-0.0, 0.0))
            self._curRawGyroAngle = self._simPose.rotation() + noise
        else:
            # Real Gyro
            # Read the value from the hardware
            self._curRawGyroAngle = self._getGyroAngle()

        # Update the WPILib Pose Estimate
        self._poseEst.update(self._curRawGyroAngle, curModulePositions)
        self._curEstPose = self._poseEst.getEstimatedPosition()

        # Record the estimate to telemetry/logging-
        self._telemetry.update(self._curEstPose, [x.angle for x in curModulePositions])

        # Remember the module positions for next loop
        self._lastModulePositions = curModulePositions

    def getCurEstPose(self)->Pose2d:
        """
        Returns:
            Pose2d: The most recent estimate of where the robot is at
        """
        return self._curEstPose
    
    def setUseAprilTags(self, use:bool):
        """
        Enables or disables pose estimate correction based on apriltag readings.
        Useful if the robot is known to be doing something (like tilting) where
        the pose estimate will go inaccurate
        """
        self._useAprilTags = use

    # Local helper to wrap the real hardware angle into a Rotation2d
    def _getGyroAngle(self)->Rotation2d:
        return Rotation2d().fromDegrees(self._gyro.getAngle(self._gyro.getYawAxis()))