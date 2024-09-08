import wpilib
import ntcore as nt
import wpiutil.log as wpilog  # pylint: disable=import-error,no-name-in-module
from utils.extDriveManager import ExtDriveManager
from utils.singleton import Singleton

BASE_TABLE = "SmartDashboard"


# Wrangler for coordinating the set of all signals
class SignalWrangler(metaclass=Singleton):
    # Starts up logging to file, along with network tables infrastructure
    # Picks appropriate logging directory based on our current target
    def __init__(self):
        # Default to publishing things under Shuffleboard, which makes things more available
        self.table = nt.NetworkTableInstance.getDefault().getTable(BASE_TABLE)
        self.publishedSigDict = {}
        self.fileLogging = False
        self.time = int(0)

        if ExtDriveManager().isConnected():
            wpilib.DataLogManager.start(dir=ExtDriveManager().getLogStoragePath())
            wpilib.DataLogManager.logNetworkTables(
                False
            )  # We have a lot of things in NT that don't need to be logged
            self.log = wpilib.DataLogManager.getLog()
            self.fileLogging = True

    def markLoopStart(self):
        self.time = nt._now()  # pylint: disable=W0212

    def publishValue(self, name, value, units):
        #global sampIdx
    
        if not name in self.publishedSigDict:
            # New signal found!

            # Set up NT publishing
            sigTopic = self.table.getDoubleTopic(name)
            sigPub = sigTopic.publish(
                nt.PubSubOptions(sendAll=True, keepDuplicates=True)
            )
            sigPub.setDefault(0)

            if(units is not None):
                sigTopic.setProperty("units", str(units))

            # Set up log file publishing if enabled
            if self.fileLogging:
                sigLog = wpilog.DoubleLogEntry(
                    log=self.log, name=sigNameToNT4TopicName(name)
                )
            else:
                sigLog = None

            # Remember handles for both
            self.publishedSigDict[name] = (sigPub, sigLog)

        # Publish value to NT
        self.publishedSigDict[name][0].set(value, self.time)
        # Put value to log file
        if self.fileLogging:
            self.publishedSigDict[name][1].append(value, self.time)



###########################################
# Public API
###########################################

_singletonInst = SignalWrangler() # cache a reference
# Log a new named value
def log(name, value, units=None):
    _singletonInst.publishValue(name, value, units)


def sigNameToNT4TopicName(name):
    return f"/{BASE_TABLE}/{name}"
