import numpy as np
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import Trajectory, TrajectoryConfig, TrajectoryGenerator
from wpilib import Timer
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS, MAX_TRANSLATE_ACCEL_MPS2
from drivetrain.drivetrainControl import DrivetrainCommand
from threading import Thread
from time import sleep

from navigation.costMapTelemetry import CostMapTelemetry
from navigation.gradientDescentCostMap import GradientDescentCostMap
from utils.calibration import Calibration

class DynamicPathPlannerGoal():
    def __init__(self, endPose:Pose2d):
        self.endPose = endPose
        self.baseMap = GradientDescentCostMap(self.endPose)
        self.baseMap.add_obstacle(Translation2d(5.56, 2.74),100,0.75)
        self.baseMap.add_obstacle(Translation2d(3.45, 4.07),100,0.75)
        self.baseMap.add_obstacle(Translation2d(5.56, 5.35),100,0.75)
        self.baseMap.add_obstacle(Translation2d(11.0, 2.74),100,0.75)
        self.baseMap.add_obstacle(Translation2d(13.27, 4.07),100,0.75)
        self.baseMap.add_obstacle(Translation2d(11.0, 5.35),100,0.75)

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
        self.pathSpeedFactor = Calibration("Dynamic Path Planner Speed Scaling", 0.75, "frac", minVal=0.0, maxVal=1.0)
        self.obstacles:list[ObstacleObservation] = []
        self.curGoal = GOAL_PICKUP
        self.replanNeeded = False
        self.pathReady = False
        self.startPose:Pose2d|None = None
        self.curTraj:Trajectory|None = None
        self.waypoints:list[Translation2d]|None = None
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
        self.pathReady = False

    def setNoGoal(self):
        self.replanNeeded = False
        self.pathReady = False

    def addObstacleObservation(self, obs:ObstacleObservation):
        self.obstacles.append(obs)
        self.replanNeeded = True
        self.pathReady = False

    @staticmethod
    def _obstacleCostHeuristic(timeToLive:float)->float:
        """ return the "height" used in pathplanning based on the alive time"""
        return timeToLive * 2.0
    
    def _getHolonomicRotation(self, time_s:float)->Rotation2d:
        if(self.curTraj is not None and self.trajStartTime_s is not None and self.startPose is not None):
            startRot = self.startPose.rotation()
            endRot = self.curGoal.endPose.rotation()
            deltaRot = endRot - startRot
            frac = (time_s)/self.curTraj.totalTime()
            if(frac > 1.0): frac = 1.0
            if(frac < 0.0): frac = 0.0
            return startRot + deltaRot*frac
        else:
            return Rotation2d() # derp


    def get(self) -> DrivetrainCommand:
        if(self.curTraj is not None and self.trajStartTime_s is not None and self.pathReady):
            # Trajectory doesn't work great for holonomic drivetrains.
            # We'll sample the path before and after the current timestanp
            # and use finite differences to derive a drivetrain velocity command
            curTime = Timer.getFPGATimestamp() - self.trajStartTime_s
            if(curTime < self.curTraj.totalTime()):
                prevTime = max(0.0, curTime - 0.01)
                nextTime = curTime + 0.01
                deltaTime = nextTime - prevTime

                prevSampleTrans = self.curTraj.sample(prevTime).pose.translation()
                curSampleTrans = self.curTraj.sample(curTime).pose.translation()
                nextSampleTrans = self.curTraj.sample(nextTime).pose.translation()
                
                # Adjust the sample poses to have the correct holonomic rotation
                prevSample = Pose2d(prevSampleTrans, self._getHolonomicRotation(prevTime))
                curSample = Pose2d(curSampleTrans,  self._getHolonomicRotation(curTime))
                nextSample = Pose2d(nextSampleTrans,  self._getHolonomicRotation(nextTime))

                xVel = (nextSample.translation().X() - prevSample.translation().X())/deltaTime
                yVel = (nextSample.translation().Y() - prevSample.translation().Y())/deltaTime
                rotVel = (nextSample.rotation() - prevSample.rotation()).radians()/deltaTime

                return DrivetrainCommand(
                    xVel,yVel,rotVel,curSample
                )
            else:
                return DrivetrainCommand(
                    0,0,0,self.curGoal.endPose
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
                self.pathReady = True
            self.telem.update(workingGrid)

    def _do_replan(self, curPose:Pose2d) -> np.ndarray:

        
        self.trajCfg = TrajectoryConfig(MAX_FWD_REV_SPEED_MPS * self.pathSpeedFactor.get(), 
                                        MAX_TRANSLATE_ACCEL_MPS2 * self.pathSpeedFactor.get())


        workingMap = self.curGoal.baseMap.get_copy()

        self.startPose = curPose

        # Handle obstacle updates
        for obs in self.obstacles:

            if(obs.deadtime_s is None):
                # Set a deadtime if we have none
                obs.deadtime_s = Timer.getFPGATimestamp() + obs.lifetime_s

            # Add obstacle to the map
            timeToLive = Timer.getFPGATimestamp() - obs.deadtime_s
            if(timeToLive > 0):
                workingMap.add_obstacle(obs.pose.translation(),
                                        self._obstacleCostHeuristic(timeToLive),
                                        obs.radius_m)
        
        # Calc a new path
        self.waypoints = workingMap.calculate_path(curPose)

        # Get start/end poses for feeding to the trajectory generation
        # These should be in the same translations as start/end points
        # But with rotations that reflect the intended velocity vector of the robot
        # at the start and end. 
        # This is needed because the trajectory generator is designed around tank drives
        # but since we have a swerve drive, robot heading and velocity vector heading are decoupled
        if(len(self.waypoints) > 2):
            startTrajPose = Pose2d(
                translation=curPose.translation(),
                rotation=self._getDirection(self.waypoints[0], self.waypoints[1])
            )
            endTrajPose = Pose2d(
                translation=self.curGoal.endPose.translation(),
                rotation=self._getDirection( self.waypoints[-2],self.waypoints[-1])
            )
        else:
            startTrajPose = Pose2d(
                translation=curPose.translation(),
                rotation=self._getDirection(curPose.translation(), 
                                            self.curGoal.endPose.translation())
            )
            endTrajPose = Pose2d(
                translation=self.curGoal.endPose.translation(),
                rotation=self._getDirection(curPose.translation(),
                                            self.curGoal.endPose.translation())
            )


        # TODO - initial velocity
        self.curTraj = TrajectoryGenerator.generateTrajectory(startTrajPose, self.waypoints[1:-2], endTrajPose, self.trajCfg)

        
        self.trajStartTime_s = Timer.getFPGATimestamp()
        return workingMap.base_grid
    
    @staticmethod
    def _getDirection(from_tans:Translation2d, to_trans:Translation2d) -> Rotation2d:
        deltaX = (to_trans.X() - from_tans.X())
        deltaY = (to_trans.Y() - from_tans.Y())
        return Rotation2d(x=deltaX, y=deltaY)

