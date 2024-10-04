import math
from wpimath.geometry import Translation2d

from navigation.navForce import Force, _logistic_func

# Generic and specifc types of obstacles that repel a robot
class Obstacle:
    def __init__(self, strength:float=1.0, radius:float = 0.25):
        self.strength = strength
        self.forceIsPositive = True

        # Force uses a logistic function to calculate push magnitiude as a fucntion of distance from center.
        # The radius offsets the center to get a larger or smaller obstacle
        # The number is in meters, and represents the half-strength point on the logistic function
        self.radius = radius

        # Parameter to logistic funciton for how steeply the force transitions from "none" to "lots" about the radius
        self.fieldSteepness = 3.5

    def setForceInverted(self, isInverted)->None:
        self.forceIsPositive = not isInverted

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
    def __init__(self, location:Translation2d, strength:float=1.0, radius:float = 0.3):
        self.location = location
        super().__init__(strength,radius)
        
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
    def __init__(self, y:float, strength:float=1.0, radius:float = 0.5):
        self.y=y
        super().__init__(strength,radius)


    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(0, self._distToForceMag(self.y - position.Y()))

    def getDist(self, position: Translation2d) -> float:
        return abs(position.y - self.y)
    
    def getTelemTrans(self) -> list[Translation2d]:
        return []
        
class VerticalObstacle(Obstacle):
    def __init__(self, x:float, strength:float=1.0, radius:float = 0.5):
        self.x=x
        super().__init__(strength,radius)

    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(self._distToForceMag(self.x - position.X()), 0)

    def getDist(self, position: Translation2d) -> float:
        return abs(position.x - self.x)

# Describes a field force that exists along a line segment with a start and end point
class LinearObstacle(Obstacle):
    def __init__(self, start:Translation2d, end:Translation2d, strength:float=0.75, radius:float = 0.4):
        self.start = start
        self.end = end
        super().__init__(strength,radius)

    def _shortestTransToSegment(self, point: Translation2d) -> Translation2d:
        # Vector from start to end of the segment
        segment_vector_x = self.end.X() - self.start.X()
        segment_vector_y = self.end.Y() - self.start.Y()
        
        # Segment length squared (to avoid a square root operation)
        segment_length_squared = segment_vector_x ** 2 + segment_vector_y ** 2
        
        # Vector from start of the segment to the point
        start_to_point_x = point.X() - self.start.X()
        start_to_point_y = point.Y() - self.start.Y()
        
        # Project the point onto the line (infinite line, not the segment)
        t = (start_to_point_x * segment_vector_x + start_to_point_y * segment_vector_y) / segment_length_squared
        
        # Clamp t to the range [0, 1] to restrict it to the segment
        t = max(0, min(1, t))
        
        # Calculate the closest point on the segment
        closest_point_x = self.start.X() + t * segment_vector_x
        closest_point_y = self.start.Y() + t * segment_vector_y
        
        # Return the shortest vector from the point to the closest point on the segment
        return Translation2d(closest_point_x - point.X(), closest_point_y - point.Y())

    def getDist(self, position: Translation2d) -> float:
        return self._shortestTransToSegment(position).norm()

# Field formation that pushes the robot toward and along a line between start/end
class Lane(LinearObstacle):

    def getForceAtPosition(self, position: Translation2d) -> Force:
        toSeg = self._shortestTransToSegment(position)
        toSegUnit = toSeg/toSeg.norm()

        alongSeg = (self.end - self.start)
        alongSegUnit = alongSeg/alongSeg.norm()

        forceDir = alongSegUnit * 0.75 + toSegUnit * 0.25
        forceDirUnit = forceDir/forceDir.norm()
        unitX = forceDirUnit.X()
        unitY = forceDirUnit.Y()

        dist = toSeg.norm()
        forceMag = self._distToForceMag(dist)

        return Force(unitX*forceMag, unitY*forceMag)
    
# Field formation that pushes the robot uniformly away from the line
class Wall(LinearObstacle):

    def getForceAtPosition(self, position: Translation2d) -> Force:
        toSeg = self._shortestTransToSegment(position)
        toSegUnit = toSeg/toSeg.norm()

        unitX = toSegUnit.X() * -1.0
        unitY = toSegUnit.Y() * -1.0

        dist = toSeg.norm()
        forceMag = self._distToForceMag(dist)

        return Force(unitX*forceMag, unitY*forceMag)
    
    
