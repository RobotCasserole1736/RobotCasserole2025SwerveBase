from phoenix6 import hardware, configs, signals, controls, StatusCode
from wpilib import TimedRobot
from utils.signalLogging import log
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
import time

## Wrappered WCP Kraken, Powered by Talon FX.
# Wrappers their API's into a consistent set of API's for swappability with rev hardware
# on Casserole's robots.
# References:
# https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/index.html
# https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/python/PositionClosedLoop/robot.py
# https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/python/VelocityClosedLoop/robot.py

class WrapperedKraken:
    def __init__(self, canID, name, brakeMode=False, currentLimitA=40.0):
        self.ctrl = hardware.TalonFX(canID, "rio")
        self.name = name
        self.configSuccess = False
        self.disconFault = Fault(f"Kraken {name} ID {canID} disconnected")

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.current_limits.supply_current_limit = currentLimitA
        self.cfg.current_limits.supply_current_limit_enable = True

        self.motorCurrentSig = self.ctrl.get_supply_current()
        self.motorCurrentSig.set_update_frequency(10.0)
        self.motorPosSig = self.ctrl.get_rotor_position()
        self.motorPosSig.set_update_frequency(50.0)
        self.motorVelSig = self.ctrl.get_rotor_velocity()
        self.motorVelSig.set_update_frequency(20.0)

        self.cfg.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE if brakeMode else signals.NeutralModeValue.COAST 

        self._applyCurCfg()

    def _applyCurCfg(self):
        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.ctrl.configurator.apply(self.cfg, 0.5) # type: ignore
            if status.is_ok():
                self.configSuccess = True
                break

        self.disconFault.set(not self.configSuccess)

    def setInverted(self, isInverted):
        if(isInverted):
            self.cfg.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.cfg.motor_output.inverted = signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self._applyCurCfg()


    def setPID(self, kP, kI, kD):
        self.cfg.slot0.k_p = kP
        self.cfg.slot0.k_i = kI
        self.cfg.slot0.k_d = kD
        self._applyCurCfg()

    def setPosCmd(self, posCmd, arbFF=0.0):
        """_summary_

        Args:
            posCmd (float): motor desired shaft rotations in radians
            arbFF (int, optional): _description_. Defaults to 0.
        """
        self.simActPos = posCmd
        posCmdRev = rad2Rev(posCmd)

        log(self.name + "_desPos", posCmd, "Rad")
        log(self.name + "_cmdVoltage", arbFF, "V")

        self.ctrl.set_control(controls.PositionVoltage(posCmdRev).with_slot(0).with_feed_forward(arbFF))

        log(self.name + "_supplyCurrent", self.motorCurrentSig.value_as_double, "A")

    def setVelCmd(self, velCmd, arbFF=0.0):
        """_summary_

        Args:
            velCmd (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """
        velCmdRPM = radPerSec2RPM(velCmd)
        velCmdRPS = velCmdRPM/60.0

        log(self.name + "_desVel", velCmdRPM, "RPM")
        log(self.name + "_cmdVoltage", arbFF, "V")

        self.ctrl.set_control(controls.VelocityVoltage(velCmdRPS).with_slot(0).with_feed_forward(arbFF))

        log(self.name + "_supplyCurrent", self.motorCurrentSig.value_as_double, "A")

    def setVoltage(self, outputVoltageVolts):
        log(self.name + "_cmdVoltage", outputVoltageVolts, "V")
        
        self.ctrl.set_control(controls.VoltageOut(outputVoltageVolts))

        log(self.name + "_supplyCurrent", self.motorCurrentSig.value_as_double, "A")

    def getMotorPositionRad(self):
        if(TimedRobot.isSimulation()):
            pos = self.simActPos
        else:
            if self.configSuccess:
                pos = rev2Rad(self.motorPosSig.value_as_double)
            else:
                pos = 0

        log(self.name + "_actPos", pos, "rad")

        return pos

    def getMotorVelocityRadPerSec(self):
        if self.configSuccess:
            vel = self.motorVelSig.value_as_double*60.0
        else:
            vel = 0
        log(self.name + "_motorActVel", vel, "RPM")
        return RPM2RadPerSec(vel)
