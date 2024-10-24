#import sys
import wpilib
from utils.signalLogging import addLog


'''
static getCurrent3V3() → float

    Get the current output of the 3.3V rail.

    Returns:

        The controller 3.3V rail output current value in Amps

static getCurrent5V() → float

    Get the current output of the 5V rail.

    Returns:

        The controller 5V rail output current value in Amps

static getCurrent6V() → float

    Get the current output of the 6V rail.

    Returns:

        The controller 6V rail output current value in Amps
        ution(*args, **
static getBatteryVoltage() → volts

    Read the battery voltage.

    Returns:

        The battery voltage in Volts.

static getInputCurrent() → float

    Get the input current to the robot controller.

    Returns:

        The controller input current value in Amps

static getInputVoltage() → float

    Get the input voltage to the robot controller.

    Returns:

        The controller input voltage value in Volts

'''

class PowerMonitor:
    """
    Wrapper class to log electrical statistics on robot usage
    """
    def __init__(self):
        self.powerDist = wpilib.PowerDistribution()
        #addLog("Battery current draw",self.powerDist.getTotalCurrent, "A")
        #addLog("RIO Voltage",wpilib.RobotController.getBatteryVoltage, "V")
        #addLog("Battery voltage", wpilib.RobotController.getInputVoltage, "V")

    def update(self):
        return
