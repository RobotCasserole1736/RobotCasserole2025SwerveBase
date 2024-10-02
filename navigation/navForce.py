

# Utility function for defining the shape of the force an obstacle exerts on the robot
from dataclasses import dataclass
import math


def _logistic_func(x, L, k, x0):
    """Logistic function."""
    return L / (1 + math.exp(-k * (x - x0)))

# Utility class for representing a force vector
@dataclass
class Force:
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