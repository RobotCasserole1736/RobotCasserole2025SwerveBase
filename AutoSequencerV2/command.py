from AutoSequencerV2.composer import Composer
from AutoSequencerV2.runnable import Runnable


# A command is the basic unit of an autonomous mode
# Commands are composed together into CommandGroups
# Commands are runnable for a finite period of time - they've got init, execute (periodic), and end methods
# Users should extend the Command class to add their own functionality to these init/execute/end methods
class Command(Runnable, Composer):
    def getName(self):
        return self.__class__.__name__
