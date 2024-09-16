import numpy as np
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpilib import Timer
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS, MAX_TRANSLATE_ACCEL_MPS2
from drivetrain.drivetrainControl import DrivetrainCommand
from threading import Thread
from time import sleep

from navigation.costMapTelemetry import CostMapTelemetry
from navigation.gradientDescentCostMap import GradientDescentCostMap

class DynamicPathPlannerGoal():
    def __init__(self, endPose:Pose2d):
        self.endPose = endPose
        self.baseMap = GradientDescentCostMap(self.endPose)

GOAL_PICKUP = DynamicPathPlannerGoal(Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0)))
GOAL_SPEAKER = DynamicPathPlannerGoal(Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(0.0)))
   
class ObstacleObservation():
    def __init__(self, pose:Pose2d, radius_m:float, lifetime_s:float):
        self.pose = pose
        self.radius_m = radius_m
        self.lifetime_s = lifetime_s
        self.deadtime_s:float|None = None

class DynamicPathPlanner():

    def __init__(self):
        self.obstacles:list[ObstacleObservation] = []
        self.curGoal = GOAL_PICKUP
        self.replanNeeded = False
        self.curTraj:Trajectory|None = None
        self.trajStartTime_s:float|None = None
        self.trajCfg = TrajectoryConfig(MAX_FWD_REV_SPEED_MPS, MAX_TRANSLATE_ACCEL_MPS2)
        self.replanThread = Thread(name="Nav DPP Replan Thread", target=self.replanThreadMain, daemon=True)
        self.replanThread.start()
        self.curPose = Pose2d()
        self.telem = CostMapTelemetry("Nav")


    def changeGoal(self, goal:DynamicPathPlannerGoal, curPose:Pose2d):
        self.curGoal = goal
        self.curPose = curPose
        self.replanNeeded = True

    def addObstacleObservation(self, obs:ObstacleObservation):
        self.obstacles.append(obs)
        self.replanNeeded = True

    @staticmethod
    def _obstacleCostHeuristic(timeToLive:float)->float:
        """ return the "height" used in pathplanning based on the alive time"""
        return timeToLive * 2.0
   
    def get(self) -> DrivetrainCommand:
        if(self.curTraj is not None and self.trajStartTime_s is not None):
            # Trajectory doesn't work great for holonomic drivetrains.
            # We'll sample the path before and after the current timestanp
            # and use finite differences to derive a drivetrain velocity command
            curTime = Timer.getFPGATimestamp() - self.trajStartTime_s
            prevTime = max(0.0, curTime - 0.01)
            nextTime = curTime + 0.01
            deltaTime = nextTime - prevTime

            prevSample = self.curTraj.sample(prevTime).pose
            curSample = self.curTraj.sample(curTime).pose
            nextSample = self.curTraj.sample(nextTime).pose

            xVel = (nextSample.translation().X() - prevSample.translation().X())/deltaTime
            yVel = (nextSample.translation().Y() - prevSample.translation().Y())/deltaTime
            rotVel = (nextSample.rotation() - prevSample.rotation()).radians()/deltaTime

            return DrivetrainCommand(
                xVel,yVel,rotVel,curSample
            )

        else:
            # No path, no motion
            return DrivetrainCommand()
       
    def replanThreadMain(self):
        workingGrid = None
        while(True):
            sleep(0.2)
            if(self.replanNeeded):
                workingGrid = self._do_replan(self.curPose)
                self.replanNeeded = False
            self.telem.update(workingGrid)




    def _do_replan(self, curPose:Pose2d) -> np.ndarray:

        workingMap = self.curGoal.baseMap.get_copy()

        # Handle obstacle updates
        for obs in self.obstacles:

            if(obs.deadtime_s is None):
                # Set a deadtime if we have none
                obs.deadtime_s = Timer.getFPGATimestamp() + obs.lifetime_s

            # Add obstacle to the map
            timeToLive = Timer.getFPGATimestamp() - obs.deadtime_s
            if(timeToLive > 0):
                workingMap.add_obstacle(obs.pose,
                                        self._obstacleCostHeuristic(timeToLive),
                                        obs.radius_m)
        
        # Calc a new path
        waypoints = workingMap.calculate_path(curPose)

        # TODO - initial velocity
        self.curTraj = TrajectoryGenerator.generateTrajectory(curPose, waypoints, self.curGoal.endPose, self.trajCfg)
        self.trajStartTime_s = Timer.getFPGATimestamp()

        return workingMap.base_grid
