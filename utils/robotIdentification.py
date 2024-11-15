from enum import Enum
import wpilib
from utils.faults import Fault
from utils.singleton import Singleton

"""
Specifc robots this codebase might run on.
"""
RobotTypes = Enum('RobotTypes', ['Main','Practice','TestBoard'])

class RobotIdentification(metaclass=Singleton):
    """
    While we strive for our practice robot and main/competition robots to be as identical as possible,
    that's not always the case. 
    The goal of this class is to identify which robot is currently running the code.
    The constants between practice and main robots may be different. 
    """

    def __init__(self):
        self.roboControl = wpilib.RobotController
        self.robotType = RobotTypes.Main
        self.serialFault = Fault("RoboRIO serial number not recognized")
        self._configureValue()

    def _configureValue(self):

        self.serialFault.setNoFault()

        if self.roboControl.getSerialNumber() == "030e2cb0":
            #Test to see if the RoboRio serial number is the main/"Production" bot.
            self.robotType = RobotTypes.Main 
        elif self.roboControl.getSerialNumber() == "03064e3f" or wpilib.TimedRobot.isSimulation():
            #Test to see if the RoboRio serial number is the practice bot.
            self.robotType = RobotTypes.Practice
        elif self.roboControl.getSerialNumber() == "0316b37c":
            #Test to see if the RoboRio serial number is our testboard's serial number.
            self.robotType = RobotTypes.TestBoard
        else:
            # If the Robo Rio's serial number is not equal to any of our known serial numbers, 
            # assume we are the main robot. But, throw a fault, since this is something software
            # team needs to fix.
            self.robotType = RobotTypes.Main
            self.serialFault.setFaulted()

    def _getRobotSerialNumber(self)->str:
        return self.roboControl.getSerialNumber()

    def getRobotType(self)->RobotTypes:
        """
        Return which robot we're running on right now
        """
        return self.robotType 

