import os
import wpilib
from utils.faults import Fault
from utils.singleton import Singleton


class ExtDriveManager(metaclass=Singleton):
    def __init__(self):
        self.conn = False
        self.driveAvailableFault = Fault("Logging USB Drive Not Available")

        if wpilib.RobotBase.isSimulation():
            # Silently disable in sim
            self.enableDiskLogging = False
            self.logDir = ""
        else:
            self.logDir = "/U/logs"
            try:
                if not os.path.isdir(self.logDir):
                    os.makedirs(self.logDir)
            except PermissionError as err:
                print("Logging disabled!")
                print(err)
                self.enableDiskLogging = False
                self.driveAvailableFault.setFaulted()

            self.conn = True

    def getLogStoragePath(self):
        return self.logDir

    def isConnected(self):
        return self.conn
