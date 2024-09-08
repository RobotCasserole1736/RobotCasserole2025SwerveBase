from dashboardWidgets.widgetConfig import WidgetConfig


class AutoChooser(WidgetConfig):
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(self, xPos, yPos, nt4Topic_in, modeList):
        WidgetConfig.__init__(self, nt4Topic_in + "/cur", xPos, yPos)
        self.nominalHeight = 5
        self.nominalWidth = 40
        self.isVisible = True
        self.modeNameList = modeList
        self.ntTopicDesVal = nt4Topic_in + "/des"

    def _getJsModeNameListString(self):
        retVal = "["
        retVal += ",".join([f'"{x}"' for x in self.modeNameList])
        retVal += "]"
        return retVal

    def getJSDeclaration(self):
        retStr = f"var widget{self.idx} = new AutoChooser('widget{self.idx}', '{self.name}', {self._getJsModeNameListString()}, onWidget{self.idx}ValUpdated);\n"
        retStr += f'nt4Client.publishNewTopic("{self.ntTopicDesVal}", "int");'
        return retStr

    def getJSSetData(self):
        retStr = ""
        retStr += f'if(name == "' + self.ntTopicCurVal + '"){ '
        retStr += f"    widget{self.idx}.setActualState(value)"
        retStr += "}"
        return retStr

    def getJSUpdate(self):
        return f"    widget{self.idx}.render()"

    def getJSSetNoData(self):
        return f"    widget{self.idx}.reportNoData()"

    def getJSCallback(self):
        retStr = ""
        retStr += f"function onWidget{self.idx}ValUpdated(value) {{\n"
        retStr += f'    nt4Client.addSample("{self.ntTopicDesVal}", nt4Client.getServerTime_us(), value);\n'
        retStr += "}"
        return retStr

    def getTopicSubscriptionStrings(self):
        retStr = ""
        retStr += '"' + self.ntTopicDesVal + '",'
        retStr += '"' + self.ntTopicCurVal + '",'
        return retStr
