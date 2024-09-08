from AutoSequencerV2.builtInCommands.doNothingCommand import DoNothingCommand
from AutoSequencerV2.mode import Mode


# A DoNothingMode is an autonomous mode where the robot just sits doing nothing indefinitely
class DoNothingMode(Mode):
    def __init__(self):
        # Build a reasonable name out of the specified duration
        Mode.__init__(self, f"Do Nothing")

    def getCmdGroup(self):
        # A wait mode should have only one command, jsut wait the specified duration
        return DoNothingCommand()
