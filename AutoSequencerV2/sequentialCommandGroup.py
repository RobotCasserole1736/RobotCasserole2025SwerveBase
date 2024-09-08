from AutoSequencerV2.composer import Composer
from AutoSequencerV2.runnable import Runnable


class SequentialCommandGroup(Runnable, Composer):
    def __init__(self, cmdList=None):
        self.cmdList = cmdList if cmdList else []
        self._curCmdIdx = 0

    def execute(self):
        if self._curCmdIdx < len(self.cmdList):
            # If we've got a valid command, execute it
            curCmd = self.cmdList[self._curCmdIdx]
            curCmd.execute()

            if curCmd.isDone():
                # Time to move on to the next command
                # First, Naturally end the current command
                print(f"[Auto] Ending {curCmd.getName()}")
                curCmd.end(False)
                # Move onto the next command
                self._curCmdIdx += 1
                # Init it if it exists
                if self._curCmdIdx < len(self.cmdList):
                    curCmd = self.cmdList[self._curCmdIdx]
                    print(f"[Auto] Starting {curCmd.getName()}")
                    curCmd.initialize()
                    # That's it, next loop we'll execute it.

    # Default group init - just init each command
    def initialize(self):
        self._curCmdIdx = 0
        if self._curCmdIdx < len(self.cmdList):
            # Init the first command
            curCmd = self.cmdList[self._curCmdIdx]
            print(f"[Auto] Starting {curCmd.getName()}")
            curCmd.initialize()

    # Default group end - end everything with same interrupted status
    def end(self, interrupted):
        # Only need to end the current command
        if self._curCmdIdx < len(self.cmdList):
            curCmd = self.cmdList[self._curCmdIdx]
            print(f"[Auto] Ending {curCmd.getName()}")
            curCmd.end(interrupted)

    def isDone(self):
        # We're done when we hit the end of the list
        return self._curCmdIdx >= len(self.cmdList)
    
    def getName(self):
        return "sequentialCommandGroup"

    ##################################################
    ## composition handlers

    def andThen(self, other):
        self.cmdList.append(other)
        return self
