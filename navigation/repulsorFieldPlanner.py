from dataclasses import dataclass
import math
from wpimath.geometry import Pose2d, Translation2d
from navigation.navConstants import FIELD_X_M, FIELD_Y_M

from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.navMapTelemetry import CostMapTelemetry

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

class Obstacle:
    def __init__(self, strength:float=1.0, forceIsPositive:bool=True):
        self.strength = strength
        self.forceIsPositive = forceIsPositive
    def getForceAtPosition(self, position:Translation2d)->Force:
        return Force()
    def _distToForceMag(self, dist:float)->float:
        forceMag = self.strength / (0.00001 + abs(dist**3))
        if(not self.forceIsPositive):
            forceMag *= -1.0
        return forceMag
    def getDist(self, position:Translation2d)->float:
        return float('inf')
    
class PointObstacle(Obstacle):
    def __init__(self, location:Translation2d, strength:float=1.0, forceIsPositive:bool=True):
        self.strength = strength
        self.forceIsPositive = forceIsPositive
        self.location = location

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

class HorizontalObstacle(Obstacle):
    def __init__(self, y:float, strength:float=1.0, forceIsPositive:bool=True):
        self.strength = strength
        self.forceIsPositive = forceIsPositive
        self.y=y

    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(0, self._distToForceMag(self.y - position.Y()))

    def getDist(self, position: Translation2d) -> float:
        return abs(position.y - self.y)
        
class VerticalObstacle(Obstacle):
    def __init__(self, x:float, strength:float=1.0, forceIsPositive:bool=True):
        self.strength = strength
        self.forceIsPositive = forceIsPositive
        self.x=x

    def getForceAtPosition(self, position: Translation2d) -> Force:
        return Force(self._distToForceMag(self.x - position.X()), 0)

    def getDist(self, position: Translation2d) -> float:
        return abs(position.x - self.x)

GOAL_STRENGTH = 0.65

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

class RepulsorFieldPlanner:
    def __init__(self):
       self.fixedObstacles:list[Obstacle] = []
       self.fixedObstacles.extend(FIELD_OBSTACLES)
       self.fixedObstacles.extend(WALLS)
       self.transientObstcales:list[Obstacle] = []
       self.telem = CostMapTelemetry(name="PotentialField")

    def setGoal(self, goal:Pose2d|None):
        self.goal = goal

    def add_obstcale_observaton(self, pose:Pose2d):
        obstacle = PointObstacle(location=Translation2d(pose.X() + 3,pose.Y()),strength=.5)
        self.transientObstcales.append(obstacle)
        print("Obstacle is ", obstacle)
        print(self.transientObstcales)

    def getGoalForce(self, curLocation:Translation2d) -> Force:
        if(self.goal is not None):
            displacement = self.goal.translation() - curLocation
            if(displacement == 0.0):
                return Force() # literally at goal, no force
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

    def updateTelemetry(self):
        self.telem.update(self)

    def getForceAtTrans(self, trans:Translation2d)->Force:
        goalForce = self.getGoalForce(trans)
        
        repusliveForces = []
        for obstacle in self.fixedObstacles:
            repusliveForces.append(obstacle.getForceAtPosition(trans))
        for obstacle in self.transientObstcales:
            repusliveForces.append(obstacle.getForceAtPosition(trans))

        # Calcualte sum of forces
        netForce = goalForce
        for force in repusliveForces:
            netForce += force

        return netForce
       
    def getCmd(self, curPose:Pose2d, stepSize_m:float) -> DrivetrainCommand:
        retVal = DrivetrainCommand() # Default, no command
        if(self.goal is not None):
            curTrans = curPose.translation()

            err = curTrans - self.goal.translation()

            if(err.norm() < stepSize_m*1.5):
                retVal.velT = 0.0
                retVal.velX = 0.0
                retVal.velY = 0.0
                retVal.desPose = self.goal
            else:
                netForce = self.getForceAtTrans(curPose.translation())

                # Take a step in the direction of the force
                step = Translation2d(stepSize_m*netForce.unitX(), stepSize_m*netForce.unitY()) 

                # Assemble velocity commands based on the step we took
                retVal.velX = (step.x)/0.02
                retVal.velY = (step.y)/0.02
                retVal.velT = 0.0 # TODO
                retVal.desPose = Pose2d(curTrans+step, curPose.rotation())
        
        return retVal