# Abstract class defining the methods that any command or command group which
# whishes to participate in a _composition_ must support


class Composer:
    def _optimizeCmdList(self, first, second, outType):
        # pylint: disable=import-outside-toplevel cyclic-import
        from AutoSequencerV2.command import (
            Command,
        )

        if isinstance(first, outType) and isinstance(second, outType):
            # They're both the same type - optimize to a single command list
            cmds = []
            cmds.extend(first.cmdList)
            cmds.extend(second.cmdList)
        elif isinstance(first, outType) and isinstance(second, Command):
            cmds = []
            cmds.extend(first.cmdList)
            cmds.append(second)
        elif isinstance(first, Command) and isinstance(second, outType):
            cmds = []
            cmds.append(first)
            cmds.extend(second.cmdList)
        else:
            cmds = [first, second]
        return cmds

    def andThen(self, other):
        # pylint: disable=import-outside-toplevel cyclic-import
        from .sequentialCommandGroup import (
            SequentialCommandGroup,
        )

        cmds = self._optimizeCmdList(self, other, SequentialCommandGroup)

        return SequentialCommandGroup(cmds)

    def raceWith(self, other):
        # pylint: disable=import-outside-toplevel cyclic-import
        from .raceCommandGroup import (
            RaceCommandGroup,
        )

        cmds = self._optimizeCmdList(self, other, RaceCommandGroup)

        return RaceCommandGroup(cmds)

    def alongWith(self, other):
        # pylint: disable=import-outside-toplevel cyclic-import
        from .parallelCommandGroup import (
            ParallelCommandGroup,
        )

        cmds = self._optimizeCmdList(self, other, ParallelCommandGroup)

        return ParallelCommandGroup(cmds)
