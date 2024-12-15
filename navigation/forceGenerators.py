import math
from wpimath.geometry import Translation2d, Rotation2d
from utils.constants import FIELD_X_M, FIELD_Y_M
from navigation.navForce import Force, logisticFunc

class ForceGenerator:
    """
    Generic, common class for all objects that generate forces for pathplanning on the field.
    Usually you should be using a specific type of force object, not this generic one
    """
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
        """
        Invert the force direction. 
        """
        self.forceIsPositive = not isInverted

    def getForceAtPosition(self, position:Translation2d)->Force:
        """
        Abstract implementation that assumes the object has no force. 
        Specific object types should override this
        """
        return Force()
    
    def _distToForceMag(self, dist:float)->float:
        """
        Common method for converting a distance from the force object into a force strength
        We're using the logistic function here, which (while not physically plaussable) has some
        nice properties about getting bigger near the obstacle, while further objects do not have much
        (or any) influence on the robot.
        """
        dist = abs(dist)
        forceMag = logisticFunc(-1.0 * dist, self.strength, self.fieldSteepness, -self.radius)
        if(not self.forceIsPositive):
            forceMag *= -1.0
        return forceMag
    def getDist(self, position:Translation2d)->float:
        """
        Returns the distance (in meters) between the object and a given position
        """
        return float('inf')
    def getTrans(self)->list[Translation2d]:
        """
        return all x/y positions associated with this field force generating object
        """
        return []


class PointObstacle(ForceGenerator):
    """
    A point obstacle is defined as round, centered at a specific point, with a specific radius
    It pushes the robot away radially outward from its center
    """
    def __init__(self, location:Translation2d, strength:float=1.0, radius:float = 0.3):
        self.location = location
        super().__init__(strength,radius)
        
    def getForceAtPosition(self, position: Translation2d) -> Force:
        deltaX =  self.location.x - position.x
        deltaY =  self.location.y - position.y
        dist = max(math.sqrt((deltaX)**2 + (deltaY)**2), 1e-6)
        unitX = deltaX/dist
        unitY = deltaY/dist
        forceMag = self._distToForceMag(dist)
        return Force(-1.0*unitX*forceMag, -1.0*unitY*forceMag)

    def getDist(self, position:Translation2d)->float:
        return (position - self.location).norm()

    def getTrans(self) -> list[Translation2d]:
        return [self.location]


class HorizontalObstacle(ForceGenerator):
    """
    Linear obstacles are infinite lines at a specific coordinate
    They push the robot away along a perpendicular direction
    with the specific direction determined by the force inversion.
    The Horizontal Obstacle exists parallel to the X axis, at a specific Y coordinate, and pushes parallel to the Y axis.
    """
    def __init__(self, y:float, strength:float=1.0, radius:float = 0.5):
        self.y=y
        super().__init__(strength,radius)


    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(0, self._distToForceMag(self.y - position.Y()))

    def getDist(self, position: Translation2d) -> float:
        return abs(position.y - self.y)
    
    def getTrans(self) -> list[Translation2d]:
        return [
            Translation2d(0,self.y),
            Translation2d(FIELD_X_M,self.y),
        ]
        
class VerticalObstacle(ForceGenerator):
    """
    Linear obstacles are infinite lines at a specific coordinate
    They push the robot away along a perpendicular direction
    with the specific direction determined by the force inversion.
    The Vertical Obstacle exists parallel to the Y axis, at a specific X coordinate, and pushes parallel to the X axis.
    """
    def __init__(self, x:float, strength:float=1.0, radius:float = 0.5):
        self.x=x
        super().__init__(strength,radius)

    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(self._distToForceMag(self.x - position.X()), 0)

    def getDist(self, position: Translation2d) -> float:
        return abs(position.x - self.x)
    
    def getTrans(self) -> list[Translation2d]:
        return [
            Translation2d(self.x,0),
            Translation2d(self.x,FIELD_Y_M),
        ]

class _LinearForceGenerator(ForceGenerator):
    """
    A linear force generator creates forces based on the relative position of the robot to a specific line segment
    """
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

    def getTrans(self) -> list[Translation2d]:
        return [
            self.start,
            self.end
        ]

class Lane(_LinearForceGenerator):
    """
    A lane is an attractor - it creates a field force that pulls robots toward and along it, "ejecting" them out the far end.
    It helps as a "hint" when the robot needs to navigate in a specific way around obstacles, which is not necessarily straight toward the goal.
    """
    def getForceAtPosition(self, position: Translation2d) -> Force:
        toSeg = self._shortestTransToSegment(position)
        dist = max(toSeg.norm(), 1e-6)
        toSegUnit = toSeg/dist

        alongSeg = (self.end - self.start)
        alongSegUnit = alongSeg/max(alongSeg.norm(),1e-6)

        forceDir = alongSegUnit * 0.75 + toSegUnit * 0.25
        forceDirUnit = forceDir/forceDir.norm()
        unitX = forceDirUnit.X()
        unitY = forceDirUnit.Y()

        forceMag = self._distToForceMag(dist)

        return Force(unitX*forceMag, unitY*forceMag)
    

class Wall(_LinearForceGenerator):
    """
    Walls obstacles are finite lines between specific coordinates
    They push the robot away along a perpendicular direction.
    """
    def getForceAtPosition(self, position: Translation2d) -> Force:
        toSeg = self._shortestTransToSegment(position)
        dist = max(toSeg.norm(), 1e-6)
        toSegUnit = toSeg/dist

        perpVec = (self.end - self.start).rotateBy(Rotation2d.fromDegrees(90.0))
        perpVec /= perpVec.norm()
        unitX = perpVec.X()
        unitY = perpVec.Y()

        forceMag = self._distToForceMag(dist)

        return Force(unitX*forceMag, unitY*forceMag)
    
    
