from AutoSequencerV2.composer import Composer
from AutoSequencerV2.runnable import Runnable


class ParallelCommandGroup(Runnable, Composer):
    def __init__(self, cmdList=None):
        self.cmdList = cmdList if cmdList else []
        self._cmdFinishedDict = {}

    def execute(self):
        for cmd in self.cmdList:
            if not self._cmdFinishedDict[cmd]:
                # Run each unfinished command in this group, checking for finish as we go.
                cmd.execute()
                if cmd.isDone():
                    # Naturally end the command when it is done.
                    print(f"[Auto] {cmd.getName()} finished")
                    cmd.end(False)
                    self._cmdFinishedDict[cmd] = True

    def initialize(self):
        # Set up the dictionary of commands to "finished" booleans
        self._cmdFinishedDict.clear()
        for cmd in self.cmdList:
            self._cmdFinishedDict[cmd] = False

        for cmd in self.cmdList:
            print(f"[Auto] Starting {cmd.getName()}")
            cmd.initialize()

    def end(self, interrupted):
        for cmd in self.cmdList:
            if not self._cmdFinishedDict[cmd]:
                # End all unfinished commands
                print(f"[Auto] Ending {cmd.getName()}")
                cmd.end(interrupted)

    def isDone(self):
        # We're done when every command has finished
        return all(self._cmdFinishedDict.values())
    
    def getName(self):
        return "Parallel Command Group"

    ##################################################
    ## composition handlers
