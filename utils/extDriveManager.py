import os
import wpilib
from utils.faults import Fault
from utils.singleton import Singleton


class ExtDriveManager(metaclass=Singleton):
    """
    The External Drive Manager is responsible for checking whether an external USB drive is
    available for logging purposes, and providing the path to it if so. Since the drive is 
    critical for debugging what happened during a match, a fault is raised if a drive is expected
    but not detected to be writeable.
    """
    def __init__(self):
        self.enableDiskLogging = False
        self.driveAvailableFault = Fault("Logging USB drive not available")

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
