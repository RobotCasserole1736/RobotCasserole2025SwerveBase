# Base class defining the methods that any command or command group which can be executed must support


class Runnable:
    def execute(self):
        pass

    def initialize(self):
        pass

    def end(self, interrupted):
        pass

    def isDone(self):
        return False
