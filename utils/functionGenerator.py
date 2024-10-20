import math
import wpilib
from utils.calibration import Calibration


class FunctionGenerator:
    """
    Class designed to create step and sine waveforms.
    These are useful to use as commands to closed-loop control systems in test mode
    as it's useful to tune PID controls around known, controllable waveforms.
    """
    def __init__(self, uniqueName):
        self.activeCal = Calibration(name="fg_" + uniqueName + "_active", default=0)
        self.typeCal = Calibration(
            name="fg_" + uniqueName + "_type",
            units="0=sine,1=square",
            minVal=0,
            maxVal=1,
        )
        self.freqCal = Calibration(
            name="fg_" + uniqueName + "_freq", units="Hz", default=2.0
        )
        self.ampCal = Calibration(name="fg_" + uniqueName + "_amp", default=1.0)
        self.offsetCal = Calibration(name="fg_" + uniqueName + "_offset")

        self.startTime = wpilib.Timer.getFPGATimestamp()
        self.active = False

    # Main periodic update. Expected to be called whenever the user wants another value
    def get(self):
        # Update the actie flag, recording the time at activation
        nextActive = self.activeCal.get()
        if not self.active and nextActive:
            self.startTime = wpilib.Timer.getFPGATimestamp()
        self.active = nextActive

        if self.active:
            # Active, calculate an output
            curTime = wpilib.Timer.getFPGATimestamp() - self.startTime
            baseOutput = math.sin(2 * math.pi * self.freqCal.get() * curTime)

            curType = self.typeCal.get()

            if curType == 1.0:
                # Square wave - rail the sine output
                if baseOutput > 0:
                    baseOutput = 0.5
                else:
                    baseOutput = -0.5

            return self.offsetCal.get() + self.ampCal.get() * baseOutput

        else:
            # Otherwise, inactive, no output
            return 0

    # indicates whether the function generator is currently active or not.
    def isActive(self):
        return self.activeCal.get()
