from dashboardWidgets.widgetConfig import WidgetConfig
from utils.signalLogging import sigNameToNT4TopicName


# Utility signal name calculation functions
def getAzmthDesTopicName(modName):
    return f"DtModule_{modName}_azmthDes"


def getAzmthActTopicName(modName):
    return f"DtModule_{modName}_azmthAct"


def getSpeedDesTopicName(modName):
    return f"DtModule_{modName}_speedDes"


def getSpeedActTopicName(modName):
    return f"DtModule_{modName}_speedAct"


# Private helper class: Describes a set of topics associated with one module
class _ModuleTopicSet:
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(self, modName, modIdx):
        self.azmthDesTopic = sigNameToNT4TopicName(getAzmthDesTopicName(modName))
        self.azmthActTopic = sigNameToNT4TopicName(getAzmthActTopicName(modName))
        self.speedDesTopic = sigNameToNT4TopicName(getSpeedDesTopicName(modName))
        self.speedActTopic = sigNameToNT4TopicName(getSpeedActTopicName(modName))
        self.modIdx = modIdx

    def getSubscriptionStrings(self):
        retStr = ""
        retStr += f'"{self.azmthDesTopic}",'
        retStr += f'"{self.azmthActTopic}",'
        retStr += f'"{self.speedDesTopic}",'
        retStr += f'"{self.speedActTopic}",'
        return retStr

    def getJSSetData(self, widgetIdx):
        retStr = ""
        retStr += f'if(name == "{self.azmthDesTopic}") {{\n'
        retStr += f"    widget{widgetIdx}.setVal({self.modIdx}, 0, value)\n"
        retStr += f"}}\n"
        retStr += f'if(name == "{self.azmthActTopic}") {{\n'
        retStr += f"    widget{widgetIdx}.setVal({self.modIdx}, 1, value)\n"
        retStr += f"}}\n"
        retStr += f'if(name == "{self.speedDesTopic}") {{\n'
        retStr += f"    widget{widgetIdx}.setVal({self.modIdx}, 2, value)\n"
        retStr += f"}}\n"
        retStr += f'if(name == "{self.speedActTopic}") {{\n'
        retStr += f"    widget{widgetIdx}.setVal({self.modIdx}, 3, value)\n"
        retStr += f"}}\n"
        return retStr


# Public class to describe a swerve state widget for the swerve drive module states
class SwerveState(WidgetConfig):
    # This code-generation class has some long lines
    # that I don't know of a good way to get rid of.
    # pylint: disable=line-too-long

    def __init__(self, xPos, yPos):
        WidgetConfig.__init__(self, None, xPos, yPos)
        self.nominalHeight = 20
        self.nominalWidth = 20
        self.isVisible = True
        self.topicsFL = _ModuleTopicSet("FL", 0)
        self.topicsFR = _ModuleTopicSet("FR", 1)
        self.topicsBL = _ModuleTopicSet("BL", 2)
        self.topicsBR = _ModuleTopicSet("BR", 3)

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new SwerveState('widget{self.idx}', '{self.name}')\n"

    def getTopicSubscriptionStrings(self):
        retStr = ""
        retStr += self.topicsFL.getSubscriptionStrings()
        retStr += self.topicsFR.getSubscriptionStrings()
        retStr += self.topicsBL.getSubscriptionStrings()
        retStr += self.topicsBR.getSubscriptionStrings()
        return retStr

    def getJSSetData(self):
        retStr = ""
        retStr += self.topicsFL.getJSSetData(self.idx)
        retStr += self.topicsFR.getJSSetData(self.idx)
        retStr += self.topicsBL.getJSSetData(self.idx)
        retStr += self.topicsBR.getJSSetData(self.idx)
        return retStr

    def getJSUpdate(self):
        return f"    widget{self.idx}.render()"
