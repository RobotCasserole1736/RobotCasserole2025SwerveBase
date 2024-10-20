from dataclasses import dataclass
from typing import Callable
import wpilib
import ntcore as nt
import wpiutil.log as wpilog  # pylint: disable=import-error,no-name-in-module
from utils.extDriveManager import ExtDriveManager
from utils.singleton import Singleton


BASE_TABLE = "SmartDashboard"

@dataclass
class _LoggedVal():
    """
    Container for holding a data source (a callable in our code)
    and all the places the data might go - NetworkTables for live publishing,
    or a file for later review. If writing files is not possible (USB drive not in?)
    the file publisher should be None
    """
    valGetter:Callable[[], float]
    ntPublisher:nt.DoublePublisher
    filePublisher:wpilog.DoubleLogEntry|None

# Wrangler for coordinating the set of all signals
class SignalWrangler(metaclass=Singleton):
    # Starts up logging to file, along with network tables infrastructure
    # Picks appropriate logging directory based on our current target
    def __init__(self):
        # Default to publishing things under Shuffleboard, which makes things more available
        self.table = nt.NetworkTableInstance.getDefault().getTable(BASE_TABLE)
        self.loggedValList:list[_LoggedVal] = []
        self.time = int(0)
        self.log = None
    
        if ExtDriveManager().isConnected():
            wpilib.DataLogManager.start(dir=ExtDriveManager().getLogStoragePath())
            wpilib.DataLogManager.logNetworkTables(
                False
            )  # We have a lot of things in NT that don't need to be logged
            self.log = wpilib.DataLogManager.getLog()

    def update(self):
        curTime = nt._now()  # pylint: disable=W0212
        for lv in self.loggedValList:
            val = lv.valGetter()
            lv.ntPublisher.set(val, curTime)
            if(lv.filePublisher is not None):
                lv.filePublisher.append(val, curTime)


    def newLogVal(self, name:str, valGetter:Callable[[],float], units:str|None):

            # Set up NT publishing
            sigTopic = self.table.getDoubleTopic(name)
            sigPub = sigTopic.publish(
                nt.PubSubOptions(sendAll=True, keepDuplicates=True)
            )
            sigPub.setDefault(0)

            if(units is not None):
                sigTopic.setProperty("units", str(units))

            # Set up log file publishing if enabled
            if self.log is not None:
                sigLog = wpilog.DoubleLogEntry(
                    log=self.log, name=sigNameToNT4TopicName(name)
                )
            else:
                sigLog = None

            self.loggedValList.append(
                _LoggedVal(valGetter,sigPub, sigLog)
            )




###########################################
# Public API
###########################################

_singletonInst = SignalWrangler() # cache a reference
def logUpdate():
    """
    Periodic call to sample and broadcast all logged values. Should happen once per 
    20ms loop.
    """
    _singletonInst.update()

def addLog(alias: str, value_getter: Callable[[], float], units=None) -> None:
    """
    Register some value to be loggd

    Parameters:
    - alias: The name used to identify the log.
    - value_getter: A function that returns the current value of the log. Lambda is acceptable here.
    """
    _singletonInst.newLogVal(alias, value_getter, units)

def sigNameToNT4TopicName(name):
    return f"/{BASE_TABLE}/{name}"
