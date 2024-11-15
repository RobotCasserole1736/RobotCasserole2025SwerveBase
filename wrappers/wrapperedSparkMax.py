from rev import CANSparkMax, SparkMaxPIDController, REVLibError, CANSparkLowLevel
from wpilib import TimedRobot
from utils.signalLogging import addLog
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
import time


_StatusFramePeriodConfigs = [
    [CANSparkMax.PeriodicFrame.kStatus0, 200],  # Status 0 = Motor output and Faults
    [
        CANSparkMax.PeriodicFrame.kStatus1,
        100,
    ],  # Status 1 = Motor velocity & electrical data
    [CANSparkMax.PeriodicFrame.kStatus2, 20],  # Status 2 = Motor Position
    [CANSparkMax.PeriodicFrame.kStatus3, 32767],  # Status 3 = Analog Sensor Input
    [CANSparkMax.PeriodicFrame.kStatus4, 32767],  # Status 4 = Alternate Encoder Stats
    [CANSparkMax.PeriodicFrame.kStatus5, 32767],  # Status 5 = Duty Cycle Encoder pt1
    [CANSparkMax.PeriodicFrame.kStatus6, 32767],  # Status 5 = Duty Cycle Encoder pt2
]


## Wrappered Spark Max
# Wrappers REV's libraries to add the following functionality for spark max controllers:
# Grouped PID controller, Encoder, and motor controller objects
# Physical unit conversions into SI units (radians)
# Retry logic for initial configuration
# Fault handling for not crashing code if the motor controller is disconnected
# Fault annunication logic to trigger warnings if a motor couldn't be configured
class WrapperedSparkMax:
    def __init__(self, canID, name, brakeMode=False, currentLimitA=40.0):
        self.ctrl = CANSparkMax(canID, CANSparkLowLevel.MotorType.kBrushless)
        self.pidCtrl = self.ctrl.getPIDController()
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

        # Perform motor configuration, tracking errors and retrying until we have success
        retryCounter = 0
        while not self.configSuccess and retryCounter < 10:
            retryCounter += 1
            errList = []
            errList.append(self.ctrl.restoreFactoryDefaults())
            mode = (
                CANSparkMax.IdleMode.kBrake
                if brakeMode
                else CANSparkMax.IdleMode.kCoast
            )
            errList.append(self.ctrl.setIdleMode(mode))
            errList.append(self.ctrl.setSmartCurrentLimit(int(currentLimitA)))

            # Apply all status mode configs
            for cfg in _StatusFramePeriodConfigs:
                errList.append(self.ctrl.setPeriodicFramePeriod(cfg[0], cfg[1]))

            # Check if any operation triggered an error
            if any(x != REVLibError.kOk for x in errList):
                print(
                    f"Failure configuring Spark Max {name} CAN ID {canID}, retrying..."
                )
                self.configSuccess = False
            else:
                # Only attempt other communication if we're able to successfully configure
                self.configSuccess = True
            time.sleep(0.1)
        
        self.disconFault.set(not self.configSuccess)

        #addLog(self.name + "_outputCurrent", self.ctrl.getOutputCurrent, "A")
        #addLog(self.name + "_appliedOutput", self.ctrl.getAppliedOutput, "%")
        #addLog(self.name + "_desVolt", lambda: self.desVolt, "V")
        #addLog(self.name + "_desPos", lambda: self.desPos, "rad")
        #addLog(self.name + "_desVel", lambda: self.desVel, "RPM")
        #addLog(self.name + "_actPos", lambda: self.actPos, "rad")
        #addLog(self.name + "_actVel", lambda: self.actVel, "RPM")


    def setInverted(self, isInverted):
        if self.configSuccess:
            self.ctrl.setInverted(isInverted)

    def setPID(self, kP, kI, kD):
        if self.configSuccess:
            self.pidCtrl.setP(kP)
            self.pidCtrl.setI(kI)
            self.pidCtrl.setD(kD)

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
            err = self.pidCtrl.setReference(
                posCmdRev,
                CANSparkMax.ControlType.kPosition,
                0,
                arbFF,
                SparkMaxPIDController.ArbFFUnits.kVoltage,
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
            err = self.pidCtrl.setReference(
                self.desVel,
                CANSparkMax.ControlType.kVelocity,
                0,
                arbFF,
                SparkMaxPIDController.ArbFFUnits.kVoltage,
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
