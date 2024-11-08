import math
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import (
    MAX_FWD_REV_SPEED_MPS,
    MAX_ROTATE_SPEED_RAD_PER_SEC,
)
#from drivetrain.controlStrategies.autoDrive import AutoDrive
from jormungandr.choreoTrajectory import ChoreoTrajectoryState
from utils.calibration import Calibration
from utils.signalLogging import addLog
from utils.mathUtils import limit

class HolonomicDriveController:
    """
    Closed-loop controller suite to get the robot from where it is to where it isn't
    https://www.youtube.com/watch?v=bZe5J8SVCYQ
    Used to emulate driver commands while following a trajectory or auto-driving.

    This is often called a "Holonomic Drive Controller" or "HDC".

    Note that wpilib has one of these, but it doesn't (yet) include feed-forward on rotation (??????)
    So we made our own.
    """

    def __init__(self, name:str):
        self.curVx = 0
        self.curVy = 0
        self.curVtheta = 0

        self.transP = Calibration(f"{name} HDC Translation kP", 6.0)
        self.transI = Calibration(f"{name} HDC Translation kI", 0.0)
        self.transD = Calibration(f"{name} HDC Translation kD", 0.0)
        self.rotP = Calibration(f"{name} HDC Rotation kP", 2.0)
        self.rotI = Calibration(f"{name} HDC Rotation kI", 0.0)
        self.rotD = Calibration(f"{name} HDC Rotation kD", .05)

        self.xFF = 0.0
        self.yFF = 0.0
        self.tFF = 0.0
        self.xFB = 0.0
        self.yFB = 0.0
        self.tFB = 0.0

        addLog(f"{name} HDC xFF", lambda:self.xFF, "mps")
        addLog(f"{name} HDC yFF", lambda:self.yFF, "mps")
        addLog(f"{name} HDC tFF", lambda:self.tFF, "radpersec")
        addLog(f"{name} HDC xFB", lambda:self.xFB, "mps")
        addLog(f"{name} HDC yFB", lambda:self.yFB, "mps")
        addLog(f"{name} HDC tFB", lambda:self.tFB, "radpersec")

        # Closed-loop control for the X position
        self.xCtrl = PIDController(
            self.transP.get(),
            self.transI.get(),
            self.transD.get(),
        )

        # Closed-loop control for the Y position
        self.yCtrl = PIDController(
            self.transP.get(),
            self.transI.get(),
            self.transD.get(),
        )

        # Closed-loop control for rotation (Theta)
        self.tCtrl = PIDController(
            self.rotP.get(),
            self.rotI.get(),
            self.rotD.get(),
        )
        # Make sure the controller knows that -170 and 170 are just 20 degrees apart
        self.tCtrl.enableContinuousInput(-math.pi, math.pi)

    def updateCals(self):
        self.xCtrl.setPID(self.transP.get(), self.transI.get(), self.transD.get())
        self.yCtrl.setPID(self.transP.get(), self.transI.get(), self.transD.get())
        self.tCtrl.setPID(self.rotP.get(), self.rotI.get(), self.rotD.get())

    def update(self, trajCmd: ChoreoTrajectoryState, curEstPose):
        """Main periodic update, call this whenever you need new commands

        Args:
            trajCmd (PathPlannerState): Current trajectory state
            curEstPose (Pose2d): Current best-estimate of where the robot is at on the field

        Returns:
            ChassisSpeeds: the Field-relative set of vx, vy, and vt commands for
            the robot to follow that will get it to the desired pose
        """
        # Feed-Forward - calculate how fast we should be going at this point in the trajectory
        xFF = trajCmd.velocityX
        yFF = trajCmd.velocityY
        tFF = trajCmd.angularVelocity
        cmdPose = trajCmd.getPose()
        return self.update2(xFF,yFF,tFF,cmdPose,curEstPose)

    def update2(self, xFF, yFF, tFF, cmdPose:Pose2d, curEstPose:Pose2d):
        # Feed-Back - Apply additional correction if we're not quite yet at the spot on the field we
        #             want to be at.
        self.xFB = self.xCtrl.calculate(curEstPose.X(), cmdPose.X())
        self.yFB = self.yCtrl.calculate(curEstPose.Y(), cmdPose.Y())
        self.tFB = self.tCtrl.calculate(
            curEstPose.rotation().radians(), cmdPose.rotation().radians()
        )

        # Remember feed-forward value inputs
        self.xFF = xFF 
        self.yFF = yFF 
        self.tFF = tFF 

        retVal = DrivetrainCommand()
        retVal.velX = limit(xFF + self.xFB, MAX_FWD_REV_SPEED_MPS)
        retVal.velY = limit(yFF + self.yFB, MAX_FWD_REV_SPEED_MPS)
        retVal.velT = limit(tFF + self.tFB, MAX_ROTATE_SPEED_RAD_PER_SEC)
        retVal.desPose = cmdPose

        return retVal
