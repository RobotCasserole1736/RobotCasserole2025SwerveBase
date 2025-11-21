from rev import SparkMax, SparkBase, SparkMaxConfig, REVLibError, ClosedLoopSlot, SparkBaseConfig, ClosedLoopConfig, SparkClosedLoopController
from wpilib import TimedRobot
from utils.signalLogging import addLog
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
import time


## Wrappered Spark Max
# Wrappers REV's libraries to add the following functionality for spark max controllers:
# Grouped PID controller, Encoder, and motor controller objects
# Physical unit conversions into SI units (radians)
# Retry logic for initial configuration
# Fault handling for not crashing code if the motor controller is disconnected
# Fault annunication logic to trigger warnings if a motor couldn't be configured
class WrapperedSparkMax:
    def __init__(self, canID, name, brakeMode=False, currentLimitA=40.0):
        self.ctrl = SparkMax(canID, SparkMax.MotorType.kBrushless)
        self.closedLoopCtrl = self.ctrl.getClosedLoopController()
        self.encoder = self.ctrl.getEncoder()
        self.name = name
        self.configSuccess = False
        self.disconFault = Fault(f"Spark Max {name} ID {canID} disconnected")
        self.simActPos = 0

        self.desPos = 0
        self.desVel = 0
        self.desVolt = 0
        self.actPos = 0
        self.actVel = 0

        self.cfg = SparkMaxConfig()
        self.cfg.signals.appliedOutputPeriodMs(200)
        self.cfg.signals.busVoltagePeriodMs(200)
        self.cfg.signals.primaryEncoderPositionPeriodMs(20)
        self.cfg.signals.primaryEncoderVelocityPeriodMs(200)
        self.cfg.setIdleMode(SparkBaseConfig.IdleMode.kBrake if brakeMode else SparkBaseConfig.IdleMode.kCoast)
        self.cfg.smartCurrentLimit(round(currentLimitA))

        # Perform motor configuration, tracking errors and retrying until we have success
        # Clear previous configuration, and persist anything set in this config.
        retryCounter = 0
        while not self.configSuccess and retryCounter < 10:
            retryCounter += 1
            err = self.ctrl.configure(self.cfg, 
                                      SparkBase.ResetMode.kResetSafeParameters, 
                                      SparkBase.PersistMode.kPersistParameters)

            # Check if any operation triggered an error
            if err != REVLibError.kOk:
                print(
                    f"Failure configuring Spark Max {name} CAN ID {canID}, retrying..."
                )
                self.configSuccess = False
            else:
                # Only attempt other communication if we're able to successfully configure
                self.configSuccess = True
            time.sleep(0.1)
        
        self.disconFault.set(not self.configSuccess)

        addLog(self.name + "_outputCurrent", self.ctrl.getOutputCurrent, "A")
        addLog(self.name + "_desVolt", lambda: self.desVolt, "V")
        addLog(self.name + "_desPos", lambda: self.desPos, "rad")
        addLog(self.name + "_desVel", lambda: self.desVel, "RPM")
        addLog(self.name + "_actPos", lambda: self.actPos, "rad")
        addLog(self.name + "_actVel", lambda: self.actVel, "RPM")


    def setInverted(self, isInverted):
        if self.configSuccess:
            self.cfg.inverted(isInverted)
            self.ctrl.configure(self.cfg,
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kPersistParameters)

    def setPID(self, kP, kI, kD, persist=SparkBase.PersistMode.kPersistParameters):
        if self.configSuccess:
            self.cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark max.
            self.ctrl.configure(self.cfg, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                persist)
            
    def setPosCmd(self, posCmd, arbFF=0.0):
        """_summary_

        Args:
            posCmd (float): motor desired shaft rotations in radians
            arbFF (int, optional): _description_. Defaults to 0.
        """
        self.simActPos = posCmd
        posCmdRev = rad2Rev(posCmd)

        self.desPos = posCmd
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                posCmdRev,
                SparkMax.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )

            self.disconFault.set(err != REVLibError.kOk)



    def setVelCmd(self, velCmd, arbFF=0.0):
        """_summary_

        Args:
            velCmd (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """

        self.desVel = radPerSec2RPM(velCmd)
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                self.desVel,
                SparkMax.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.disconFault.set(err != REVLibError.kOk)

    def setVoltage(self, outputVoltageVolts):
        self.desVolt = outputVoltageVolts
        if self.configSuccess:
            self.ctrl.setVoltage(outputVoltageVolts)

    def getMotorPositionRad(self):
        if(TimedRobot.isSimulation()):
            pos = self.simActPos
        else:
            if self.configSuccess:
                pos = rev2Rad(self.encoder.getPosition())
            else:
                pos = 0
        self.actPos = pos
        return pos

    def getMotorVelocityRadPerSec(self):
        if self.configSuccess:
            vel = self.encoder.getVelocity()
        else:
            vel = 0
        self.actVel = vel
        return RPM2RadPerSec(vel)
