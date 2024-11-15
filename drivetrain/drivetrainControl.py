from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Pose2d, Rotation2d
from Autonomous.commands.driveForwardSlowCommand import DriveForwardSlowCommand
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from drivetrain.swerveModuleControl import SwerveModuleControl
from drivetrain.swerveModuleGainSet import SwerveModuleGainSet
from drivetrain.drivetrainPhysical import (
    FL_ENCODER_MOUNT_OFFSET_RAD,
    MAX_FWD_REV_SPEED_MPS,
    FR_ENCODER_MOUNT_OFFSET_RAD,
    BL_ENCODER_MOUNT_OFFSET_RAD,
    BR_ENCODER_MOUNT_OFFSET_RAD,
    kinematics,
)
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from utils.singleton import Singleton
from utils.allianceTransformUtils import onRed
from utils.constants import (DT_FL_WHEEL_CANID, 
                             DT_FL_AZMTH_CANID, 
                             DT_FR_WHEEL_CANID, 
                             DT_FR_AZMTH_CANID, 
                             DT_BL_WHEEL_CANID, 
                             DT_BL_AZMTH_CANID,
                             DT_BR_WHEEL_CANID,
                             DT_BR_AZMTH_CANID,
                             DT_FL_AZMTH_ENC_PORT,
                             DT_FR_AZMTH_ENC_PORT,
                             DT_BL_AZMTH_ENC_PORT,
                             DT_BR_AZMTH_ENC_PORT)

class DrivetrainControl(metaclass=Singleton):
    """
    Top-level control class for controlling a swerve drivetrain
    """

    def __init__(self):
        self.modules = []
        self.modules.append(
            SwerveModuleControl("FL", DT_FL_WHEEL_CANID, DT_FL_AZMTH_CANID, DT_FL_AZMTH_ENC_PORT, 
                                FL_ENCODER_MOUNT_OFFSET_RAD, True, True)
        )
        self.modules.append(
            SwerveModuleControl("FR", DT_FR_WHEEL_CANID, DT_FR_AZMTH_CANID, DT_FR_AZMTH_ENC_PORT, 
                                FR_ENCODER_MOUNT_OFFSET_RAD, True, True)
        )
        self.modules.append(
            SwerveModuleControl("BL", DT_BL_WHEEL_CANID, DT_BL_AZMTH_CANID, DT_BL_AZMTH_ENC_PORT, 
                                BL_ENCODER_MOUNT_OFFSET_RAD, False, True)
        )
        self.modules.append(
            SwerveModuleControl("BR", DT_BR_WHEEL_CANID, DT_BR_AZMTH_CANID, DT_BR_AZMTH_ENC_PORT, 
                                BR_ENCODER_MOUNT_OFFSET_RAD, False, True)
        )

        self.desChSpd = ChassisSpeeds()
        self.curDesPose = Pose2d()
        self.curManCmd = DrivetrainCommand()
        self.curCmd = DrivetrainCommand()

        self.gains = SwerveModuleGainSet()

        self.poseEst = DrivetrainPoseEstimator(self.getModulePositions())

        self._updateAllCals()

    def setManualCmd(self, cmd: DrivetrainCommand):
        """Send commands to the robot for motion relative to the field

        Args:
            cmd (DrivetrainCommand): manual command input
        """
        self.curManCmd = cmd

    def update(self):
        """
        Main periodic update, should be called every 20ms
        """
        curEstPose = self.poseEst.getCurEstPose()

        # Iterate through all strategies for controlling the drivetrain to
        # calculate the current drivetrain commands.

        self.curCmd = self.curManCmd
        self.curCmd = Trajectory().update(self.curCmd, curEstPose)
        self.curCmd = AutoDrive().update(self.curCmd, curEstPose)

        # Transform the current command to be robot relative
        tmp = ChassisSpeeds.fromFieldRelativeSpeeds(
            self.curCmd.velX, self.curCmd.velY, self.curCmd.velT, curEstPose.rotation()
        )
        self.desChSpd = _discretizeChSpd(tmp)

        # Set the desired pose for telemetry purposes
        self.poseEst._telemetry.setDesiredPose(self.curCmd.desPose)

        # Given the current desired chassis speeds, convert to module states
        desModStates = kinematics.toSwerveModuleStates(self.desChSpd)

        # Scale back commands if one corner of the robot is going too fast
        kinematics.desaturateWheelSpeeds(desModStates, MAX_FWD_REV_SPEED_MPS)

        # Send commands to modules and update
        for idx, module in enumerate(self.modules):
            module.setDesiredState(desModStates[idx])
            module.update()

        # Update the estimate of our pose
        self.poseEst.update(self.getModulePositions(), self.getModuleStates())

        # Update calibration values if they've changed
        if self.gains.hasChanged():
            self._updateAllCals()

    def _updateAllCals(self):
        # Helper function - updates all calibration on request
        for module in self.modules:
            module.setClosedLoopGains(self.gains)

    def getModulePositions(self):
        """
        Returns:
            Tuple of the actual module positions (as read from sensors)
        """
        return tuple(mod.getActualPosition() for mod in self.modules)
    
    def getModuleDesStates(self):
        """
        Returns:
            Tuple of the desired module states (as read from sensors)
        """
        return tuple(mod.getDesiredState() for mod in self.modules)

    def getModuleStates(self):
        """
        Returns:
            Tuple of the actual module speeds (as read from sensors)
        """
        return tuple(mod.getActualState() for mod in self.modules)

    def resetGyro(self):
        # Update pose estimator to think we're at the same translation,
        # but aligned facing downfield
        curTranslation = self.poseEst.getCurEstPose().translation()
        newGyroRotation = (
            Rotation2d.fromDegrees(180.0) if (onRed()) else Rotation2d.fromDegrees(0.0)
        )
        newPose = Pose2d(curTranslation, newGyroRotation)
        self.poseEst.setKnownPose(newPose)

    def getCurEstPose(self) -> Pose2d:
        # Return the current best-guess at our pose on the field.
        return self.poseEst.getCurEstPose()


def _discretizeChSpd(chSpd):
    """See https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
        Corrects for 2nd order kinematics
        Should be included in wpilib 2024, but putting here for now

    Args:
        chSpd (ChassisSpeeds): ChassisSpeeds input

    Returns:
        ChassisSpeeds: Adjusted ch speed
    """
    dt = 0.02
    poseVel = Pose2d(chSpd.vx * dt, chSpd.vy * dt, Rotation2d(chSpd.omega * dt))
    twistVel = Pose2d().log(poseVel)
    return ChassisSpeeds(twistVel.dx / dt, twistVel.dy / dt, twistVel.dtheta / dt)
