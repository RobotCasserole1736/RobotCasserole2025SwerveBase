from AutoSequencerV2.composer import Composer
from AutoSequencerV2.runnable import Runnable


class RaceCommandGroup(Runnable, Composer):
    def __init__(self, cmdList=None):
        self.cmdList = cmdList if cmdList else []
        # Set to the index of the command which finished first, or None if all are running
        self._finishedFirstIdx = None

    def execute(self):
        if not self.isDone():
            for idx, cmd in enumerate(self.cmdList):
                # Run each command in this group, checking for finish as we go.
                cmd.execute()
                if cmd.isDone():
                    # If we're finished, stop updating
                    print(f"[Auto] {cmd.getName()} finished first")
                    self._finishedFirstIdx = idx
                    break

    # Default group init - just init each command
    def initialize(self):
        self._finishedFirstIdx = None

        # Init all the cmds
        for cmd in self.cmdList:
            print(f"[Auto] Starting {cmd.getName()}")
            cmd.initialize()

    # Default group end - end everything with same interrupted status
    def end(self, interrupted):
        # Finish each child command
        for idx, cmd in enumerate(self.cmdList):
            isInterrupted = (
                False if idx == self._finishedFirstIdx else True or interrupted
            )
            print(f"[Auto] Ending {cmd.getName()}")
            cmd.end(isInterrupted)

    def isDone(self):
        # We're done when one command has finished
        return self._finishedFirstIdx is not None
    
    def getName(self):
        return "raceCommandGroup"

    ##################################################
    ## composition handlers

    def raceWith(self, other):
        self.cmdList.append(other)
        return self
