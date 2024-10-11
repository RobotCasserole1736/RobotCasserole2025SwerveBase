from wpilib import PWMMotorController
from utils.constants import LED_STACK_LIGHT_CTRL_PWM
from utils.singleton import Singleton
from wpimath.filter import Debouncer

BLINK = -1.0
GREEN = 0.35
RED = 0.03
BLUE = 0.85
OFF = 0.0

class LEDControl(metaclass=Singleton):
    def __init__(self):

        self._isAutoDrive = False
        self._isStuck = False
        self.stuckDebounce = Debouncer(0.3, Debouncer.DebounceType.kFalling)
        self.ctrlBlock = PWMMotorController("LEDCtrl", LED_STACK_LIGHT_CTRL_PWM)

    def update(self):

        stuckDebounced = self.stuckDebounce.calculate(self._isStuck)

        if(self._isAutoDrive):
            if(stuckDebounced):
                pwmVal = RED * BLINK
            else:
                pwmVal = BLUE
        else:
            pwmVal = GREEN

        self.ctrlBlock.set(pwmVal)

    def setAutoDrive(self, isAutoDrive:bool):
        self._isAutoDrive = isAutoDrive

    def setStuck(self, isStuck:bool):
        self._isStuck = isStuck