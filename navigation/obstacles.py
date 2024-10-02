import math
from wpimath.geometry import Translation2d

from navigation.navForce import Force, _logistic_func

# Generic and specifc types of obstacles that repel a robot
class Obstacle:
    def __init__(self, strength:float=1.0, forceIsPositive:bool=True, radius:float = 0.25):
        self.strength = strength
        self.forceIsPositive = forceIsPositive

        # Force uses a logistic function to calculate push magnitiude as a fucntion of distance from center.
        # The radius offsets the center to get a larger or smaller obstacle
        # The number is in meters, and represents the half-strength point on the logistic function
        self.radius = radius

        # Parameter to logistic funciton for how steeply the force transitions from "none" to "lots" about the radius
        self.fieldSteepness = 3.5

    def getForceAtPosition(self, position:Translation2d)->Force:
        return Force()
    
    def _distToForceMag(self, dist:float)->float:
        dist = abs(dist)
        #Sigmoid shape
        forceMag = _logistic_func(-1.0 * dist, self.strength, self.fieldSteepness, -self.radius)
        if(not self.forceIsPositive):
            forceMag *= -1.0
        return forceMag
    def getDist(self, position:Translation2d)->float:
        return float('inf')
    def getTelemTrans(self)->list[Translation2d]:
        return []


# A point obstacle is defined as round, centered at a specific point, with a specific radius
# It pushes the robot away radially outward from its center
class PointObstacle(Obstacle):
    def __init__(self, location:Translation2d, strength:float=1.0, forceIsPositive:bool=True, radius:float = 0.3):
        self.location = location
        super().__init__(strength, forceIsPositive,radius)
        
    def getForceAtPosition(self, position: Translation2d) -> Force:
        deltaX =  self.location.x - position.x
        deltaY =  self.location.y - position.y
        dist = math.sqrt((deltaX)**2 + (deltaY)**2)
        unitX = deltaX/dist
        unitY = deltaY/dist
        forceMag = self._distToForceMag(dist)
        return Force(-1.0*unitX*forceMag, -1.0*unitY*forceMag)
    def getDist(self, position:Translation2d)->float:
        return (position - self.location).norm()
    def getTelemTrans(self) -> list[Translation2d]:
        return [self.location]


# Linear obstacles are infinite lines at a specific coordinate
# They push the robot away along a perpendicular direction
# with the specific direction determined by forceIsPositive
class HorizontalObstacle(Obstacle):
    def __init__(self, y:float, strength:float=1.0, forceIsPositive:bool=True, radius:float = 0.5):
        self.y=y
        super().__init__(strength, forceIsPositive,radius)


    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(0, self._distToForceMag(self.y - position.Y()))

    def getDist(self, position: Translation2d) -> float:
        return abs(position.y - self.y)
    
    def getTelemTrans(self) -> list[Translation2d]:
        return []
        
class VerticalObstacle(Obstacle):
    def __init__(self, x:float, strength:float=1.0, forceIsPositive:bool=True, radius:float = 0.5):
        self.x=x
        super().__init__(strength, forceIsPositive,radius)

    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(self._distToForceMag(self.x - position.X()), 0)

    def getDist(self, position: Translation2d) -> float:
        return abs(position.x - self.x)