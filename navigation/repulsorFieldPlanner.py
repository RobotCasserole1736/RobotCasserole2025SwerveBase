from dataclasses import dataclass
import math
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from navigation.navConstants import FIELD_X_M, FIELD_Y_M

from drivetrain.drivetrainCommand import DrivetrainCommand
from utils.mapLookup2d import MapLookup2D
from utils.signalLogging import log


# Utility function for defining the shape of the force an obstacle exerts on the robot
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

# Relative strength of how hard the goal pulls the robot toward it
# Too big and the robot will be pulled through obstacles
# Too small and the robot will get stuck on obstacles ("local minima")
GOAL_STRENGTH = 0.04

# The Fixed obstacles are everything fixed on the field, plus the walls
FIELD_OBSTACLES = [
    PointObstacle(location=Translation2d(5.56, 2.74)),
    PointObstacle(location=Translation2d(3.45, 4.07)),
    PointObstacle(location=Translation2d(5.56, 5.35)),
    PointObstacle(location=Translation2d(11.0, 2.74)),
    PointObstacle(location=Translation2d(13.27, 4.07)),
    PointObstacle(location=Translation2d(11.0, 5.35)),
]

WALLS = [
   HorizontalObstacle(y=0.0, forceIsPositive=True),
   HorizontalObstacle(y=FIELD_Y_M, forceIsPositive=False),
   VerticalObstacle(x=0.0, forceIsPositive=True),
   VerticalObstacle(x=FIELD_X_M, forceIsPositive=False)
]

# This map controls how the commanded velocity slows down as we approach the goal
GOAL_MARGIN_M = 0.05
SLOW_DOWN_DISTANCE_M = 1.5
GOAL_SLOW_DOWN_MAP = MapLookup2D([
    (9999.0, 1.0),
    (SLOW_DOWN_DISTANCE_M, 1.0),
    (GOAL_MARGIN_M, 0.0),
    (0.0, 0.0)
])

# These define how far in advance we attempt to plan for telemetry purposes
TELEM_LOOKAHEAD_DIST_M = 3.0
TELEM_LOOKAHEAD_STEPS = 7

class RepulsorFieldPlanner:
    def __init__(self):
        self.fixedObstacles:list[Obstacle] = []
        self.fixedObstacles.extend(FIELD_OBSTACLES)
        self.fixedObstacles.extend(WALLS)
        self.transientObstcales:list[Obstacle] = []
        self.distToGo:float = 0.0
        self.goal:Pose2d|None = None

    def setGoal(self, goal:Pose2d|None):
        self.goal = goal

    def add_obstcale_observaton(self, pose:Pose2d):
        obstacle = PointObstacle(location=pose.translation(),strength=.7)
        self.transientObstcales.append(obstacle)

    def getGoalForce(self, curLocation:Translation2d) -> Force:
        if(self.goal is not None):
            displacement = self.goal.translation() - curLocation
            direction = displacement/displacement.norm()
            mag = GOAL_STRENGTH * (1 + 1.0/(0.0001 + displacement.norm()))
            return Force(direction.x*mag, direction.y*mag)
        else:
            # no goal, no force
            return Force()

    def decay_observations(self):
        # Linear decay of each transient obstacle observation
        for obs in self.transientObstcales:
            obs.strength -= 0.01

        # Only keep obstacles with positive strength
        self.transientObstcales = [x for x in self.transientObstcales if x.strength > 0.0]

    def getObstacleTransList(self) -> list[Translation2d]:
        retArr = []
        for obstacle in self.fixedObstacles:
            retArr.extend(obstacle.getTelemTrans())
        for obstacle in self.transientObstcales:
            retArr.extend(obstacle.getTelemTrans())

        return retArr
    
    
    def atGoal(self, trans:Translation2d)->bool:
        if(self.goal is None):
            return True
        else:
            return (self.goal.translation() - trans).norm() < GOAL_MARGIN_M

    def getForceAtTrans(self, trans:Translation2d)->Force:

        goalForce = self.getGoalForce(trans)
        
        repusliveForces = []

        for obstacle in self.fixedObstacles:
            repusliveForces.append(obstacle.getForceAtPosition(trans))
        for obstacle in self.transientObstcales:
            repusliveForces.append(obstacle.getForceAtPosition(trans))

        # calculate sum of forces
        netForce = goalForce
        for force in repusliveForces:
            netForce += force

        return netForce
    

    def getCmd(self, curPose:Pose2d, stepSize_m:float) -> DrivetrainCommand:
        retVal = DrivetrainCommand() # Default, no command

        if(self.goal is not None):
            curTrans = curPose.translation()
            self.distToGo = (curTrans - self.goal.translation()).norm()

            if(not self.atGoal(curTrans)):
                # Only calculate a nonzero command if we have a goal and we're not near it.

                # Slow down when we're near the goal
                slowFactor = GOAL_SLOW_DOWN_MAP.lookup(self.distToGo)

                netForce = self.getForceAtTrans(curPose.translation())

                # Calculate a substep in the direction of the force
                step = Translation2d(stepSize_m*netForce.unitX(), stepSize_m*netForce.unitY()) 

                # Take that substep
                nextTrans = curTrans + step

                # Assemble velocity commands based on the step we took
                retVal.velX = (nextTrans - curTrans).X()/0.02 * slowFactor
                retVal.velY = (nextTrans - curTrans).Y()/0.02 * slowFactor
                retVal.velT = 0.0 # Let the closed-loop controller do the work.
                retVal.desPose = Pose2d(nextTrans, self.goal.rotation())
        else:
            self.distToGo = 0.0

        return retVal
    
    def updateTelemetry(self, curPose:Pose2d) -> list[Pose2d]:        
        telemTraj = []
        stepsize = TELEM_LOOKAHEAD_DIST_M/TELEM_LOOKAHEAD_STEPS
        if(self.goal is not None):
            cp = curPose
            for _ in range(0,TELEM_LOOKAHEAD_STEPS):
                telemTraj.append(cp)
                tmp = self.getCmd(cp, stepsize)
                if(tmp.desPose is not None):
                    cp = tmp.desPose
                else:
                    break

        log("PotentialField Num Obstacles", len(self.fixedObstacles) + len(self.transientObstcales))
        log("PotentialField Path Active", self.goal is not None)
        log("PotentialField DistToGo", self.distToGo, "m")
        return telemTraj
