

# Utility function for defining the shape of the force an obstacle exerts on the robot
from dataclasses import dataclass
import math


def logisticFunc(x, L, k, x0):
    """
    Implements the Logistic function.
    https://en.wikipedia.org/wiki/Logistic_function
    This function is nice due to being exactly 0 at one side, 1.0 at the other, and a smooth transition between the two
    """
    try:
        return L / (1 + math.exp(-k * (x - x0)))
    except OverflowError:
        return 0.0

@dataclass
class Force:
    """
    Simple class to represent a force in a 2d plane. Represents the metaphorical "forces"
    we use to perform on-the-fly path generation
    """
    x:float=0
    y:float=0
    def __add__(self, other):
        return Force(self.x+other.x, self.y+other.y)
    def unitX(self) -> float:
        return self.x/self.mag()
    def unitY(self) -> float:
        return self.y/self.mag()
    def mag(self):
        return math.sqrt(self.x**2 + self.y**2)