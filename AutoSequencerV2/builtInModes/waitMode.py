from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.mode import Mode


# A WaitMode is an autonomous mode where the robot just sits doing nothing for a specified duration.
class WaitMode(Mode):
    def __init__(self, duration):
        # Build a reasonable name out of the specified duration
        Mode.__init__(self, f"Wait {duration}s")
        self._duration = duration

    def getCmdGroup(self):
        # A wait mode should have only one command, jsut wait the specified duration
        return WaitCommand(self._duration)
