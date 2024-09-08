import math
from wpilib import DigitalInput, DutyCycle
from utils.faults import Fault
from utils.signalLogging import log
from utils.calibration import Calibration
from utils.units import wrapAngleRad


class WrapperedPulseWidthEncoder:
    """
    Wrappers any absolute-angle encoder which encodes angle into pulse width
    Assumes the encoder has been connected to a DIO port on the RoboRIO
    Reads the absolute angle via pulse duration
    Includes logging and handling fault detection
    """

    def __init__(
        self,
        port,
        name,
        mountOffsetRad,
        dirInverted,
        minPulse,
        maxPulse,
        minAcceptableFreq,
    ):
        self.dutyCycle = DutyCycle(DigitalInput(port))
        self.name = f"Encoder_{name}"
        self.disconFault = Fault(f"{self.name} DIO port {port} Disconnected")
        self.mountOffsetCal = Calibration(
            self.name + "_mountOffset", mountOffsetRad, "rad"
        )
        self.faulted = False
        self.curAngleRad = 0
        self.dirInverted = dirInverted

        self.minPulseTimeSec = minPulse
        self.maxPulseTimeSec = maxPulse
        self.minAcceptableFreq = minAcceptableFreq

    def update(self):
        """Return the raw angle reading from the sensor in radians"""
        freq = self.dutyCycle.getFrequency()
        self.faulted = (
            freq < self.minAcceptableFreq
        )  # abnormal frequency, we must be faulted
        self.disconFault.set(self.faulted)

        if self.faulted:
            # Faulted - don't do any processing
            pulseTime = -1
            rawAngle = 0.0
            self.curAngleRad = 0.0
        else:
            # Not-Faulted - read the raw angle from the pulse width
            pulseTime = self.dutyCycle.getOutput() * (1.0 / freq)
            rawAngle = (
                (
                    (pulseTime - self.minPulseTimeSec)
                    / (self.maxPulseTimeSec - self.minPulseTimeSec)
                )
                * 2
                * math.pi
            )

            # Invert, Offset, and wrap the reading as needed
            if self.dirInverted:
                rawAngle *= -1.0

            self.curAngleRad = wrapAngleRad(rawAngle - self.mountOffsetCal.get())

        log(f"{self.name}_freq", freq, "Hz")
        log(f"{self.name}_pulseTime", pulseTime, "sec")
        log(f"{self.name}_angle", self.curAngleRad, "rad")

    def getAngleRad(self):
        return self.curAngleRad

    def isFaulted(self):
        return self.faulted
