from dashboardWidgets.widgetConfig import WidgetConfig


class Icon(WidgetConfig):
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    # Mirror the state definitions from JS
    kOFF = 0
    kON = 1
    kBLINK_FAST = 2
    kBLINK_SLOW = 3

    def __init__(self, xPos, yPos, nt4Topic_in, colorOn, symbolName):
        WidgetConfig.__init__(self, nt4Topic_in, xPos, yPos)
        self.nominalHeight = 5
        self.nominalWidth = 5
        self.colorOn = colorOn
        self.symbolPath = 'icons/' + symbolName + '.svg'
        self.isVisible = True

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new Icon('widget{self.idx}', '{self.name}','{self.colorOn}','{self.symbolPath}')\n"

    def getJSSetData(self):
        retStr = ""
        retStr += 'if(name == "' + self.ntTopicCurVal + '"){ \n'
        retStr += "    if(value == 1){ \n"
        retStr += f"        widget{self.idx}.setVal(Icon.kON);\n"
        retStr += "    } else if(value == 2) {\n"
        retStr += f"        widget{self.idx}.setVal(Icon.kBLINK_FAST);\n"
        retStr += "    } else if(value == 3) {\n"
        retStr += f"        widget{self.idx}.setVal(Icon.kBLINK_SLOW);\n"
        retStr += "    } else {\n"
        retStr += f"        widget{self.idx}.setVal(Icon.kOFF);\n"
        retStr += "    }"
        retStr += "}"
        return retStr

    def getJSUpdate(self):
        return f"    widget{self.idx}.render()"

    def getJSSetNoData(self):
        return f"    widget{self.idx}.reportNoData()"
