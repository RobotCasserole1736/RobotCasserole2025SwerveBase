# Base class for any dashboard widget and its configuration
class WidgetConfig:
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(self, ntTopic, xPos, yPos):
        self.name = ""
        self.idx = 0
        self.ntTopicCurVal = ntTopic
        self.isVisible = False
        self.xPos = xPos
        self.yPos = yPos
        self.sizeScaleFactor = 1.0
        self.nominalWidth = 0.0
        self.nominalHeight = 0.0

    def getTopicSubscriptionStrings(self):
        return '"' + self.ntTopicCurVal + '",'

    def getHTML(self):
        if self.isVisible:
            height = self.nominalHeight * self.sizeScaleFactor
            width = self.nominalWidth * self.sizeScaleFactor
            retstr = '<div class="widgetBase" style="top:' + str(self.yPos - height / 2)
            retstr += "%;left:" + str(self.xPos - width / 2)
            retstr += "%;height:" + str(height) + "vw;width:" + str(width)
            retstr += 'vw" id="widget' + str(self.idx) + '"></div>'
            return retstr
        else:
            return ""

    def getJSDeclaration(self):
        return ""

    def getJSCallback(self):
        return ""

    def getJSSetData(self):
        retStr = ""
        retStr += 'if(name == "' + self.ntTopicCurVal + '"){ '
        retStr += f"    widget{self.idx}.setVal(value)"
        retStr += "}"
        return retStr

    def getJSUpdate(self):
        return f"    widget{self.idx}.render()"

    def getJSSetNoData(self):
        return f"    widget{self.idx}.reportNoData()"
