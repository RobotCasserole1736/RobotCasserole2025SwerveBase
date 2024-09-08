from dashboardWidgets.widgetConfig import WidgetConfig


class LineGauge(WidgetConfig):
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(
        self, xPos, yPos, nt4Topic_in, minRange, maxRange, minAcceptable, maxAcceptable
    ):
        WidgetConfig.__init__(self, nt4Topic_in, xPos, yPos)
        self.nominalHeight = 5
        self.nominalWidth = 20
        self.minRange = minRange
        self.maxRange = maxRange
        self.minAcceptable = minAcceptable
        self.maxAcceptable = maxAcceptable
        self.isVisible = True

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new LineGauge('widget{self.idx}', '{self.name}',{self.minRange},{self.maxRange},{self.minAcceptable}, {self.maxAcceptable})\n"
