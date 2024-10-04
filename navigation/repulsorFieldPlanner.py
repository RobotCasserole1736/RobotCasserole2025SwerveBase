from dataclasses import dataclass
import math
from wpimath.geometry import Pose2d, Translation2d, Transform2d, Rotation2d
from navigation.navConstants import FIELD_X_M, FIELD_Y_M

from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.navForce import Force
from navigation.obstacles import HorizontalObstacle, Obstacle, PointObstacle, VerticalObstacle
from utils.mapLookup2d import MapLookup2D
from utils.signalLogging import log

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

    def add_obstcale_observaton(self, obs:Obstacle):
        self.transientObstcales.append(obs)

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

                nextTrans = curTrans

                for _ in range(4):

                    if (nextTrans - curTrans).norm() >= stepSize_m:
                        break

                    netForce = self.getForceAtTrans(nextTrans)

                    # Calculate a substep in the direction of the force
                    step = Translation2d(stepSize_m*netForce.unitX()*0.5, stepSize_m*netForce.unitY()*0.5) 

                    # Take that step
                    nextTrans += step


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
