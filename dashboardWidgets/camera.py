import wpilib
from dashboardWidgets.widgetConfig import WidgetConfig


def getRIOStreamURL(port):
    if wpilib.RobotBase.isSimulation():
        return f"http://localhost:{port}"
    else:
        return f"http://roborio-1736-frc.local:{port}"


class Camera(WidgetConfig):
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(self, xPos, yPos, streamURL):
        WidgetConfig.__init__(self, "", xPos, yPos)
        self.nominalHeight = 25
        self.nominalWidth = 30
        self.isVisible = True
        self.streamURL = streamURL

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new Camera('widget{self.idx}', '{self.name}', '{self.streamURL}')\n"

    def getJSSetData(self):
        return ""

    def getJSUpdate(self):
        return ""

    def getJSSetNoData(self):
        return ""
