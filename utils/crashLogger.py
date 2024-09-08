import os
import logging
from datetime import datetime
import wpilib

from utils.extDriveManager import ExtDriveManager


class CrashLogger:

    """
    Python code has many more issues which are caught at runtime. In case one of these happens while on the field,
    it's important that we record what happened. This class adds an extra logging handle to record these to uniquely
    named log files on the USB drive for later retrieval
    """

    def update(self):
        if (
            not self.prefixWritten
            and wpilib.DriverStation.isFMSAttached()
            and self.isRunning
        ):
            self.logPrint(f"==========================================")
            self.logPrint(f"== FMS Data Received {datetime.now()}:")
            self.logPrint(f"Event: {wpilib.DriverStation.getEventName()}")
            self.logPrint(f"Match Type: {wpilib.DriverStation.getMatchType()}")
            self.logPrint(f"Match Number: {wpilib.DriverStation.getMatchNumber()}")
            self.logPrint(f"Replay Number: {wpilib.DriverStation.getReplayNumber()}")
            self.logPrint(
                f"Game Message: {wpilib.DriverStation.getGameSpecificMessage()}"
            )
            self.logPrint(f"Cur FPGA Time: {wpilib.Timer.getFPGATimestamp()}")
            self.logPrint(f"==========================================")
            self.flushPrint()
            self.prefixWritten = True


    def logPrint(self, msg):
        self.fileHandler.stream.write(msg)
        self.fileHandler.stream.write("\n")

    def flushPrint(self):
        self.fileHandler.stream.flush()

    def __init__(self):
        self.prefixWritten = False
        self.isRunning = ExtDriveManager().isConnected()

        if self.isRunning:
            # Iterate till we got a unique log name
            idx = 0
            uniqueFileFound = False
            logPath = ""
            while not uniqueFileFound:
                logFileName = f"crashLog_{idx}.log"
                logPath = os.path.join(
                    ExtDriveManager().getLogStoragePath(), logFileName
                )
                uniqueFileFound = not os.path.isfile(logPath)
                idx += 1

            # Install a custom logger for all errors. This should include stacktraces
            # if the robot crashes on the field.
            logFormatter = logging.Formatter(
                "%(asctime)s [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s"
            )
            rootLogger = logging.getLogger()

            self.fileHandler = logging.FileHandler(logPath)
            self.fileHandler.setFormatter(logFormatter)
            self.fileHandler.setLevel(logging.ERROR)
            rootLogger.addHandler(self.fileHandler)

            self.logPrint(f"\n==============================================")
            self.logPrint(f"Beginning of Log {logPath}")
            self.logPrint(f"Started {datetime.now()}")
