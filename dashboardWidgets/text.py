from dashboardWidgets.widgetConfig import WidgetConfig


class Text(WidgetConfig):
    def __init__(self, xPos, yPos, nt4Topic_in):
        WidgetConfig.__init__(self, nt4Topic_in, xPos, yPos)
        self.nominalHeight = 3
        self.nominalWidth = 40
        self.isVisible = True

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new Text('widget{self.idx}', '{self.name}')\n"
