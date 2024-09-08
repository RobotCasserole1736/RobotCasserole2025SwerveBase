import wpilib

from utils.signalLogging import log


# Utilties for tracking how long certain chunks of code take
# including logging overall loop execution time
class SegmentTimeTracker:
    def __init__(self, longLoopThresh=0.53):
        self.longLoopThresh = longLoopThresh
        self.tracer = wpilib.Tracer()
        self.loopStartTime = wpilib.Timer.getFPGATimestamp()
        self.loopEndTime = wpilib.Timer.getFPGATimestamp()
        self.prevLoopStartTime = self.loopStartTime
        self.curPeriod = 0
        self.curLoopExecDur = 0

    def start(self):
        self.tracer.clearEpochs()
        self.prevLoopStartTime = self.loopStartTime
        self.loopStartTime = wpilib.Timer.getFPGATimestamp()

    def mark(self, name):
        self.tracer.addEpoch(name)

    def end(self):
        self.loopEndTime = wpilib.Timer.getFPGATimestamp()
        self.curPeriod = self.loopStartTime - self.prevLoopStartTime
        self.curLoopExecDur = self.loopEndTime - self.loopStartTime
        if self.curLoopExecDur > self.longLoopThresh:
            self.tracer.printEpochs()
        log("LoopPeriod", self.curPeriod * 1000.0, "ms")
        log("LoopDuration", self.curLoopExecDur * 1000.0, "ms")
