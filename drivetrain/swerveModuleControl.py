import random

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.filter import SlewRateLimiter
from wpilib import TimedRobot


from drivetrain.swerveModuleGainSet import SwerveModuleGainSet
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSRXMagEncoder import WrapperedSRXMagEncoder
from dashboardWidgets.swerveState import getAzmthDesTopicName, getAzmthActTopicName
from dashboardWidgets.swerveState import getSpeedDesTopicName, getSpeedActTopicName
from utils.signalLogging import addLog
from drivetrain.drivetrainPhysical import dtMotorRotToLinear
from drivetrain.drivetrainPhysical import dtLinearToMotorRot
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS


class SwerveModuleControl:
    """
    Control logic for one swerve drive module. 

    Convention Reminders:
    The **module** refers to the whole assembly, including two motors, their built-in sensors, 
    the azimuth angle sensor, the hardware, everything.

    The **azimuth** is the motor, sensor, and mechanism to point the wheel in a specific direction.

    Positive azimuth rotation is counter-clockwise when viewed top-down. By the right hand rule, this is
    rotation in the positive-Z direction. Zero degrees is toward the front of the robot

    The **wheel** is the motor and mechanism to apply a force in that direction.

    Positive wheel rotation causes the robot to move forward if the azimuth is pointed forward.

    Uses WPILib convention for names:
    1) "State" refers to the speed of the wheel, plus the position of the azimuth
    2) "Position" refers to the position of the wheel, plus the position of the azimuth
    """

    def __init__(
        self,
        moduleName:str,
        wheelMotorCanID:int,
        azmthMotorCanID:int,
        azmthEncoderPortIdx:int,
        azmthOffset:float,
        invertWheel:bool,
        invertAzmth:bool
    ):
        """Instantiate one swerve drive module

        Args:
            moduleName (str): Name Prefix for the module (IE, "FL", or "BR"). For logging purposes mostly
            wheelMotorCanID (int): CAN Id for the wheel motor for this module
            azmthMotorCanID (int): CAN Id for the azimuth motor for this module
            azmthEncoderPortIdx (int): RIO Port for the azimuth absolute encoder for this module
            azmthOffset (float): Mounting offset of the azimuth encoder in Radians.
            invertWheel (bool): Inverts the drive direction of the wheel - needed since left/right sides are mirrored
            invertAzmth (bool): Inverts the steering direction of the wheel - needed if motor is mounted upside
        """
        self.wheelMotor = WrapperedSparkMax(
            wheelMotorCanID, moduleName + "_wheel", False
        )
        self.azmthMotor = WrapperedSparkMax(
            azmthMotorCanID, moduleName + "_azmth", True
        )

        # Note the azimuth encoder inversion should be fixed, based on the physical design of the encoder itself,
        # plus the swerve module physical construction. It might need to be tweaked here though if we change 
        # module brands or sensor brands.
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


        addLog(
            getAzmthDesTopicName(moduleName),
            self.optimizedDesiredState.angle.degrees,
            "deg",
        )
        addLog(
            getAzmthActTopicName(moduleName),
            self.actualState.angle.degrees,
            "deg",
        )
        addLog(
            getSpeedDesTopicName(moduleName),
            lambda: (self.optimizedDesiredState.speed / MAX_FWD_REV_SPEED_MPS),
            "frac",
        )
        addLog(
            getSpeedActTopicName(moduleName),
            lambda: ((self.actualState.speed) / MAX_FWD_REV_SPEED_MPS),
            "frac",
        )


        # Simulation Support Only
        self.wheelSimFilter = SlewRateLimiter(24.0)

    def getActualPosition(self)->SwerveModulePosition:
        """
        Returns:
            SwerveModulePosition: The position of the module (azmth and wheel) as measured by sensors
        """
        return self.actualPosition

    def getActualState(self)->SwerveModuleState:
        """
        Returns:
            SwerveModuleState: The state of the module (azmth and wheel) as measured by sensors
        """
        return self.actualState

    def getDesiredState(self)->SwerveModuleState:
        """
        Returns:
            SwerveModuleState: The commanded, desired state of the module (azmth and wheel)
        """
        return self.desiredState

    def setClosedLoopGains(self, gains:SwerveModuleGainSet):
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

    def setDesiredState(self, desState:SwerveModuleState):
        """Main command input - Call this to tell the module to go to a certian wheel speed and azimuth angle

        Args:
            desState (SwerveModuleState): The commanded state of the module
        """
        self.desiredState = desState

    def update(self):
        """Main update function, call every 20ms"""

        # Read from the azimuth angle sensor (encoder)
        self.azmthEnc.update()

        if TimedRobot.isReal():
            # Real Robot. Use the actual sensors to get data about the module.
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

        if TimedRobot.isSimulation():
            # Simulation only. Do a very rough simulation of module behavior, and populate
            # sensor data for the next loop.

            # Very simple voltage/motor model of azimuth rotation
            self.actualState.angle += Rotation2d.fromDegrees(self.azmthVoltage / 12.0 * 1000.0 * 0.02)
            self.actualPosition.angle = self.actualState.angle

            # Wheel speed is slew-rate filtered to roughly simulate robot inertia
            speed = self.wheelSimFilter.calculate(self.optimizedDesiredState.speed)
            self.actualState.speed = speed + random.uniform(-0.0, 0.0)
            self.actualPosition.distance += self.actualState.speed * 0.02
