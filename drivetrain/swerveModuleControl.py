import random

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
import wpilib


from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSRXMagEncoder import WrapperedSRXMagEncoder
from dashboardWidgets.swerveState import getAzmthDesTopicName, getAzmthActTopicName
from dashboardWidgets.swerveState import getSpeedDesTopicName, getSpeedActTopicName
from utils.signalLogging import log
from utils.units import rad2Deg
from utils.faults import Fault
from utils.robotIdentification import RobotIdentification
from drivetrain.drivetrainPhysical import dtMotorRotToLinear
from drivetrain.drivetrainPhysical import dtLinearToMotorRot
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS


class SwerveModuleControl:
    """
    Control logic for one swerve drive module
    """

    def __init__(
        self,
        moduleName,
        wheelMotorCanID,
        azmthMotorCanID,
        azmthEncoderPortIdx,
        azmthOffset,
        invertWheel,
        invertAzmth
    ):
        """Instantiate one swerve drive module

        Args:
            moduleName (str): Name Prefix for the module (IE, "FL", or "BR"). For logging purposes mostly
            wheelMotorCanID (int): CAN Id for the wheel motor for this module
            azmthMotorCanID (int): CAN Id for the azimuth motor for this module
            azmthEncoderPortIdx (int): RIO Port for the azimuth absolute encoder for this module
            azmthOffset (float): Mounting offset of the azimuth encoder in Radians.
            invertWheel (bool): Inverts the drive direction of the wheel - needed since left/right sides are mirrored
            invertWheel (bool): Inverts the steering direction of the wheel - needed if motor is mounted upside
        """
        self.wheelMotor = WrapperedSparkMax(
            wheelMotorCanID, moduleName + "_wheel", False
        )
        self.azmthMotor = WrapperedSparkMax(
            azmthMotorCanID, moduleName + "_azmth", True
        )
        self.azmthEnc = WrapperedSRXMagEncoder(
            azmthEncoderPortIdx, moduleName + "_azmthEnc", azmthOffset, False
        )

        self.wheelMotor.setInverted(invertWheel)
        self.azmthMotor.setInverted(invertAzmth)

        self.wheelMotorFF = SimpleMotorFeedforwardMeters(0, 0, 0)

        self.desiredState = SwerveModuleState()
        self.optimizedDesiredState = SwerveModuleState()
        self.actualState = SwerveModuleState()
        self.actualPosition = SwerveModulePosition()

        self.azmthCtrl = PIDController(0, 0, 0)
        self.azmthCtrl.enableContinuousInput(-180.0, 180.0)
        self.azmthVoltage = 0.0

        self._prevMotorDesSpeed = 0

        self.moduleName = moduleName

        self.serialFault = Fault(f"Serial Number Unknown")

        # Simulation Support Only
        self.wheelSimFilter = SlewRateLimiter(24.0)

    def _updateTelemetry(self):
        """
        Helper function to put all relevant data to logs and dashboards for this module
        """
        log(
            getAzmthDesTopicName(self.moduleName),
            self.optimizedDesiredState.angle.degrees(),
            "deg",
        )
        log(
            getAzmthActTopicName(self.moduleName),
            self.actualState.angle.degrees(),
            "deg",
        )
        log(
            getSpeedDesTopicName(self.moduleName),
            self.optimizedDesiredState.speed / MAX_FWD_REV_SPEED_MPS,
            "frac",
        )
        log(
            getSpeedActTopicName(self.moduleName),
            (self.actualState.speed) / MAX_FWD_REV_SPEED_MPS,
            "frac",
        )

        if RobotIdentification().getSerialFaulted():
            self.serialFault.setFaulted()
        else:
            self.serialFault.setNoFault()

    def getActualPosition(self):
        """
        Returns:
            SwerveModulePosition: The position of the module (azmth and wheel) as measured by sensors
        """
        return self.actualPosition

    def getActualState(self):
        """
        Returns:
            SwerveModuleState: The state of the module (azmth and wheel) as measured by sensors
        """
        return self.actualState

    def getDesiredState(self):
        """
        Returns:
            SwerveModuleState: The commanded, desired state of the module (azmth and wheel)
        """
        return self.desiredState

    def setClosedLoopGains(self, gains):
        """Set feed-forward and closed loop gains for the module

        Args:
            gains (SwerveModuleGainSet): The gains for this module
        """
        self.wheelMotor.setPID(
            gains.wheelP.get(), gains.wheelI.get(), gains.wheelD.get()
        )
        self.wheelMotorFF = SimpleMotorFeedforwardMeters(
            gains.wheelS.get(), gains.wheelV.get(), gains.wheelA.get()
        )
        self.azmthCtrl.setPID(
            gains.azmthP.get(), gains.azmthI.get(), gains.azmthD.get()
        )

    def setDesiredState(self, desState):
        """Main command input - Call this to tell the module to go to a certian wheel speed and azimuth angle

        Args:
            desState (SwerveModuleState): The commanded state of the module
        """
        self.desiredState = desState

    def update(self):
        """Main update function, call every 20ms"""

        # Read from the azimuth angle sensor (encoder)
        self.azmthEnc.update()

        if wpilib.TimedRobot.isReal():
            # Real Robot
            # Update this module's actual state with measurements from the sensors
            self.actualState.angle = Rotation2d(self.azmthEnc.getAngleRad())
            self.actualState.speed = dtMotorRotToLinear(
                self.wheelMotor.getMotorVelocityRadPerSec()
            )
            self.actualPosition.distance = dtMotorRotToLinear(
                self.wheelMotor.getMotorPositionRad()
            )
            self.actualPosition.angle = self.actualState.angle

        # Optimize our incoming swerve command to minimize motion
        self.optimizedDesiredState = SwerveModuleState.optimize(
            self.desiredState, self.actualState.angle
        )

        # Use a PID controller to calculate the voltage for the azimuth motor
        self.azmthCtrl.setSetpoint(self.optimizedDesiredState.angle.degrees())  # type: ignore
        self.azmthVoltage = self.azmthCtrl.calculate(self.actualState.angle.degrees())
        self.azmthMotor.setVoltage(self.azmthVoltage)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        self.optimizedDesiredState.speed *= (self.optimizedDesiredState.angle - self.actualState.angle ).cos()

        # Send voltage and speed commands to the wheel motor
        motorDesSpd = dtLinearToMotorRot(self.optimizedDesiredState.speed)
        motorDesAccel = (motorDesSpd - self._prevMotorDesSpeed) / 0.02
        motorVoltageFF = self.wheelMotorFF.calculate(motorDesSpd, motorDesAccel)
        self.wheelMotor.setVelCmd(motorDesSpd, motorVoltageFF)

        self._prevMotorDesSpeed = motorDesSpd  # save for next loop

        if wpilib.TimedRobot.isSimulation():
            # Very simple voltage/motor model of azimuth rotation
            self.actualState.angle += Rotation2d.fromDegrees(self.azmthVoltage / 12.0 * 1000.0 * 0.02)
            self.actualPosition.angle = self.actualState.angle

            # Wheel speed is slew-rate filtered to roughly simulate robot inertia
            speed = self.wheelSimFilter.calculate(self.optimizedDesiredState.speed)
            self.actualState.speed = speed + random.uniform(-0.0, 0.0)
            self.actualPosition.distance += self.actualState.speed * 0.02


        self._updateTelemetry()
