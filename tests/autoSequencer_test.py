# pylint: disable-all
from AutoSequencerV2.autoSequencer import *
from AutoSequencerV2.command import Command
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.raceCommandGroup import RaceCommandGroup


class CountingCommand(Command):
    def initialize(self):
        self.execCount = 0

    def execute(self):
        self.execCount += 1

    def isDone(self):
        return self.execCount >= 2 or self.execCount < 0

    def end(self, interrupted):
        self.execCount = -1


class CountingMode(Mode):
    def getCmdGroup(self):
        return SequentialCommandGroup([CountingCommand()])


def test_topLevel():
    dut = AutoSequencer()
    dut.addMode(CountingMode())
    dut.updateMode()
    dut.initialize()
    dut.update()
    dut.end()


def test_parallel():
    dut = CountingCommand().alongWith(CountingCommand().alongWith(CountingCommand()))

    assert isinstance(dut, ParallelCommandGroup)
    assert len(dut.cmdList) == 3
    for cmd in dut.cmdList:
        assert isinstance(cmd, Command)

    dut.initialize()

    for cmd in dut.cmdList:
        assert cmd.execCount == 0

    dut.execute()

    for cmd in dut.cmdList:
        assert cmd.execCount == 1

    dut.execute()

    for cmd in dut.cmdList:
        assert cmd.isDone()

    dut.end(False)

    # Now all commands should be ended
    for cmd in dut.cmdList:
        assert cmd.execCount == -1


def test_sequential():
    dut = CountingCommand().andThen(CountingCommand().andThen(CountingCommand()))

    assert isinstance(dut, SequentialCommandGroup)
    assert len(dut.cmdList) == 3
    for cmd in dut.cmdList:
        assert isinstance(cmd, Command)

    dut.initialize()

    # Check for init in the first command
    assert dut.cmdList[0].execCount == 0

    dut.execute()
    dut.execute()

    assert dut.cmdList[0].execCount == -1
    assert dut.cmdList[1].execCount == 0

    dut.execute()
    dut.execute()

    assert dut.cmdList[1].execCount == -1
    assert dut.cmdList[2].execCount == 0

    dut.execute()
    dut.execute()

    for cmd in dut.cmdList:
        assert cmd.isDone()


def test_race():
    dut = CountingCommand().raceWith(CountingCommand().raceWith(CountingCommand()))

    assert isinstance(dut, RaceCommandGroup)
    assert len(dut.cmdList) == 3

    dut.initialize()

    for cmd in dut.cmdList:
        assert isinstance(cmd, Command)
        assert cmd.execCount == 0  # type: ignore

    dut.execute()

    for cmd in dut.cmdList:
        assert cmd.execCount == 1

    dut.execute()

    # First command should have finished first
    assert dut.cmdList[0].isDone()
    assert not dut.cmdList[1].isDone()
    assert not dut.cmdList[2].isDone()

    dut.end(False)

    # Now all commands should be ended
    for cmd in dut.cmdList:
        assert cmd.execCount == -1
