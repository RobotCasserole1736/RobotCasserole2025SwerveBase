import math
from wpilib import DigitalInput, DutyCycle
from utils.faults import Fault
from utils.signalLogging import addLog
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
        self.disconFault = Fault(f"{self.name} DIO port {port} disconnected")
        self.mountOffsetCal = Calibration(
            self.name + "_mountOffset", mountOffsetRad, "rad"
        )
        self.faulted = False
        self.curAngleRad = 0
        self.dirInverted = dirInverted

        self.minPulseTimeSec = minPulse
        self.maxPulseTimeSec = maxPulse
        self.minAcceptableFreq = minAcceptableFreq

        self.freq = 0
        self.pulseTime = 0

        #addLog(f"{self.name}_freq", lambda: self.freq, "Hz")
        #addLog(f"{self.name}_pulseTime", lambda: self.pulseTime, "sec")
        #addLog(f"{self.name}_angle", lambda: self.curAngleRad, "rad")

    def update(self):
        """Return the raw angle reading from the sensor in radians"""
        self.freq = self.dutyCycle.getFrequency()
        self.faulted = (
            self.freq < self.minAcceptableFreq
        )  # abnormal frequency, we must be faulted
        self.disconFault.set(self.faulted)

        if self.faulted:
            # Faulted - don't do any processing
            self.pulseTime = -1
            rawAngle = 0.0
            self.curAngleRad = 0.0
        else:
            # Not-Faulted - read the raw angle from the pulse width
            self.pulseTime = self.dutyCycle.getOutput() * (1.0 / self.freq)
            rawAngle = (
                (
                    (self.pulseTime - self.minPulseTimeSec)
                    / (self.maxPulseTimeSec - self.minPulseTimeSec)
                )
                * 2
                * math.pi
            )

            # Invert, Offset, and wrap the reading as needed
            if self.dirInverted:
                rawAngle *= -1.0

            self.curAngleRad = wrapAngleRad(rawAngle - self.mountOffsetCal.get())


    def getAngleRad(self):
        return self.curAngleRad

    def isFaulted(self):
        return self.faulted
