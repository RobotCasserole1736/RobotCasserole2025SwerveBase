from dataclasses import dataclass
from enum import Enum
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import Timer

@dataclass
class DynamicPathPlannerGoal():
    endPose:Pose2d

class DynamicPathPlanGoals(Enum):
    PICKUP = DynamicPathPlannerGoal(Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0)))
    SPEAKER = DynamicPathPlannerGoal(Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(0.0)))
    
class ObstacleObservation():
    def __init__(self, pose:Pose2d, radius_m:float, lifetime_s:float):
        self.pose = pose
        self.radius_m = radius_m
        self.lifetime_s = lifetime_s
        self.deadtime_s:float|None = None

class DynamicPathPlanner():

    def __init__(self):
        self.obstacles:list[ObstacleObservation] = []
        self.curGoal = DynamicPathPlanGoals.PICKUP

    def setGoal(self, goal:DynamicPathPlannerGoal):
        self.curGoal = goal

    def addObstacleObservation(self, obs:ObstacleObservation):
        self.obstacles.append(obs)

    def periodic(self):

        # Handle obstacle updates
        for obs in self.obstacles:
            if(obs.deadtime_s is None):
                obs.deadtime_s = Timer.getFPGATimestamp() + obs.lifetime_s
            timeToLive = 

