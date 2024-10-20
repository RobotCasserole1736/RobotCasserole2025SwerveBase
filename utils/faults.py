import math
import wpilib

from utils.constants import FIX_ME_LED_PIN, HEARTBEAT_LED_PIN
from utils.singleton import Singleton


class FaultWrangler(metaclass=Singleton):
    """
    The Fault Wrangler tracks all faults, and pulses two LED's to indicate current state.
    """

    def __init__(self):
        self.faultList = []
        self.activeFaultCount = 0
        self.loopCounter = 0
        self.statusUpdateLoops = 4  # Only update the status every 40 loops
        self.curDisplayedFaultIdx = 0

    def update(self):
        """
        Main update - call this every 20ms from main robot code to keep animating LED blinks, indicating code is running
        """
        self.loopCounter += 1
        if self.loopCounter == self.statusUpdateLoops:
            # Every N loops, Update status String
            activeFaults = [x for x in self.faultList if x.isActive]
            self.activeFaultCount = len(activeFaults)
            self.loopCounter = 0  # reset counter

            if self.activeFaultCount > 0:
                self.curDisplayedFaultIdx = (
                    self.curDisplayedFaultIdx + 1
                ) % self.activeFaultCount
                curFaultString = activeFaults[self.curDisplayedFaultIdx].message
            else:
                curFaultString = ""

            wpilib.SmartDashboard.putNumber("numFaults", self.activeFaultCount)
            wpilib.SmartDashboard.putString("faultDescription", curFaultString)

        FaultStatusLEDs().update()

    def register(self, fault):
        self.faultList.append(fault)

    def hasActiveFaults(self):
        return self.activeFaultCount > 0


class FaultStatusLEDs(metaclass=Singleton):
    """
    Drives the two LED's on the robot maintenence panel based on current fault status.
    "FIX-ME" -> Red -> blinks when one or more faults are active.
    "HEARTBEAT" -> White -> pulses like a heart beat normally when code is running.

    Pit crew should be trained to look for a good heartbeat, AND no fix-me.

    No heartbeat means code isn't running. SW team needed to investigate.
    Heartbeat AND fix-me means software detected hardware broken or disconnected. The driver dashboard should be pulled up to investigate the actual root cause. 

    Admonition to software people: You must be very rigirous about not cerating false positives here. If 
    the fix-me light is on and blinking when the robot _doesn't_ need pit crew attention, they'll quickly
    learn to just ignore it. And thereby ignore any real faults that get detected.

    Unless you want the pit crew to stop and fix the robot, DON'T blink the fix-me light.
    """
    def __init__(self):
        self.fixMeLED = wpilib.DigitalOutput(FIX_ME_LED_PIN)
        self.fixMeLED.setPWMRate(500.0)
        self.fixMeLED.enablePWM(
            1.0
        )  # Initially should be just "ON" until the first call to update

        self.heartbeatLED = wpilib.DigitalOutput(HEARTBEAT_LED_PIN)
        self.heartbeatLED.setPWMRate(500.0)
        self.heartbeatLED.enablePWM(
            1.0
        )  # Initially should be just "ON" until the first call to update

    def update(self):
        # Update faults LED
        if FaultWrangler().activeFaultCount > 0:
            self.fixMeLED.updateDutyCycle(self._blinkPattern(1.3))
        else:
            self.fixMeLED.updateDutyCycle(0.0)

        # Update heartbeat LED
        self.heartbeatLED.updateDutyCycle(self._blinkPattern(0.75))

    # Returns a time-varying blink intensity to drive the LED
    # at a given frequency
    def _blinkPattern(self, freq):
        return abs(math.sin(2 * math.pi * wpilib.Timer.getFPGATimestamp() * freq / 2.0))


###########################################
# Public API
###########################################


# Create a new Fault whenever you have a condition for which you can
# annunciate a fault
class Fault:
    """
    A Fault is an abnormal condition on our robot which software can detect, and requires pit crew attention.
    """

    def __init__(self, message):
        self.message = message
        FaultWrangler().register(self)
        self.isActive = False

    def set(self, isActive):
        self.isActive = isActive

    def setFaulted(self):
        self.isActive = True

    def setNoFault(self):
        self.isActive = False
