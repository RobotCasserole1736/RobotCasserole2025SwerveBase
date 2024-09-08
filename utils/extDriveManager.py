import os
import wpilib
from utils.faults import Fault
from utils.singleton import Singleton


class ExtDriveManager(metaclass=Singleton):
    def __init__(self):
        self.enableDiskLogging = False
        self.driveAvailableFault = Fault("Logging USB Drive Not Available")

        if wpilib.RobotBase.isSimulation():
            # Disable in sim
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


        if(os.path.isdir(self.logDir)):
            self.enableDiskLogging = True
            self.driveAvailableFault.setNoFault()
        else:
            self.enableDiskLogging = False
            self.driveAvailableFault.setFaulted()


    def getLogStoragePath(self):
        return self.logDir

    def isConnected(self):
        return self.enableDiskLogging
