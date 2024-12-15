from wpilib import TimedRobot
from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from navigation.navConstants import GOAL_SPEAKER
from utils.mapLookup2d import MapLookup2D
from utils.mathUtils import limit
from utils.signalLogging import addLog
from collections import deque

from drivetrain.drivetrainCommand import DrivetrainCommand

from navigation.navForce import Force
from navigation.forceGenerators import HorizontalObstacle, ForceGenerator, PointObstacle, VerticalObstacle, Wall, Lane
from utils.constants import FIELD_X_M, FIELD_Y_M

import math

# Relative strength of how hard the goal pulls the robot toward it
# Too big and the robot will be pulled through obstacles
# Too small and the robot will get stuck on obstacles ("local minima")
GOAL_STRENGTH = 0.04

# Maximum number of obstacles we will remember. Older obstacles are automatically removed from our memory
MAX_OBSTACLES = 10

# Rate at which transient obstacles decay in strength
# Bigger numbers here make transient obstacles disappear faster
TRANSIENT_OBS_DECAY_PER_LOOP = 0.01

# Obstacles come in two main flavors: Fixed and Transient.
# Transient obstacles decay and disappear over time. They are things like other robots, or maybe gamepieces we don't want to drive through.
# Fixed obstacles are things like field elements or walls with fixed, known positions. They are always present and do not decay over time.

# Fixed Obstacles - Six posts in the middle of the field from 2024
FIELD_OBSTACLES_2024 = [
    PointObstacle(location=Translation2d(5.56, 2.74)),
    PointObstacle(location=Translation2d(3.45, 4.07)),
    PointObstacle(location=Translation2d(5.56, 5.35)),
    PointObstacle(location=Translation2d(11.0, 2.74)),
    PointObstacle(location=Translation2d(13.27, 4.07)),
    PointObstacle(location=Translation2d(11.0, 5.35))
]

# Lane Traffic pattern around center
x1 = 3.0
x2 = FIELD_X_M - x1
y1 = 2.0
y2 = FIELD_Y_M - y1

# Adds asymmetry so robot doesn't get stuck on opposite side of spaceship
FIELD_LANES_2019 = [
    Lane(Translation2d(x2 - 2.5, y2), Translation2d(x1 + 2.5, y2)),
    Lane(Translation2d(x2, y2 - 1.5), Translation2d(x2, y1 + 1.5)),
]

FIELD_OBSTACLES_2019 = [
    # Center spaceship
    Wall(Translation2d(5.6, 3.5), Translation2d(10.8, 3.5)),
    Wall(Translation2d(5.6, 3.5), Translation2d(5.6, 4.5)),
    Wall(Translation2d(5.6, 4.5), Translation2d(10.8, 4.5)),
    Wall(Translation2d(10.8, 3.5), Translation2d(10.8, 4.5)),
    # Rockets
    PointObstacle(location=Translation2d(5.8, 0.0)),
    PointObstacle(location=Translation2d(10.6, 0.0)),
    PointObstacle(location=Translation2d(5.8, FIELD_Y_M)),
    PointObstacle(location=Translation2d(10.6, FIELD_Y_M)),
    # Blue endgame platform
    Wall(Translation2d(0.0, 2.5), Translation2d(1.5, 2.5)),
    Wall(Translation2d(0.0, 5.7), Translation2d(1.5, 5.7)),
    Wall(Translation2d(1.5, 2.5), Translation2d(1.5, 5.7)),
    # Red endgame platform
    Wall(Translation2d(FIELD_X_M, 2.5), Translation2d(FIELD_X_M - 1.5, 2.5)),
    Wall(Translation2d(FIELD_X_M, 5.7), Translation2d(FIELD_X_M - 1.5, 5.7)),
    Wall(Translation2d(FIELD_X_M - 1.5, 2.5), Translation2d(FIELD_X_M - 1.5, 5.7)),
]

# Fixed Obstacles - Outer walls of the field 
B_WALL = HorizontalObstacle(y=0.0)
T_WALL = HorizontalObstacle(y=FIELD_Y_M)
T_WALL.setForceInverted(True)

L_WALL = VerticalObstacle(x=0.0)
R_WALL = VerticalObstacle(x=FIELD_X_M)
R_WALL.setForceInverted(True)

WALLS = [B_WALL, T_WALL, L_WALL, R_WALL]

# Rotation Control - we'll command rotation at this fixed rate until we're within margin
ROT_RAD_PER_SEC = 2.5
GOAL_MARGIN_DEG = 5.0

# Fixed periodic rate assumed for execution
Ts = 0.02

# Usually, the path planner assumes we traverse the path at a fixed velocity
# However, near the goal, we'd like to slow down. This map defines how we ramp down
# the step-size toward zero as we get closer to the goal. Once we are close enough,
# we stop taking steps and simply say the desired position is at the goal.
GOAL_MARGIN_M = 0.2
SLOW_DOWN_DISTANCE_M = 1.5
GOAL_SLOW_DOWN_MAP = MapLookup2D([
    (9999.0, 1.0),
    (SLOW_DOWN_DISTANCE_M, 1.0),
    (GOAL_MARGIN_M, 0.0),
    (0.0, 0.0)
])

# These define how far in advance we attempt to plan for telemetry purposes
# Increasing the distance causes us to look further ahead
LOOKAHEAD_DIST_M = 1.5
# Increasing the number of steps increases the accuracy of our look-ahead prediction
# but also increases CPU load on the RIO.
LOOKAHEAD_STEPS = 4
LOOKAHEAD_STEP_SIZE = LOOKAHEAD_DIST_M/LOOKAHEAD_STEPS

# If the lookahead routine's end poit is within this, we declare ourselves stuck.
STUCK_DIST = LOOKAHEAD_DIST_M/4

class RepulsorFieldPlanner:
    """
    This class finds a path through the FRC playing field, from current location to a defined final goal, while avoiding defined obstacles.
    The methodology is using "repulser field" or "potential field" algorithm - see
    https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf for pictures

    The basic idea: The goal provides an "attracting" force to pull the robot toward it.
    Obstacles provide a repulsive force, pushing robots away from themselves.
    Optionally (not yet fully implemented), "Lane" objects pull robots toward and along themselves

    At each step, the planner calculates the force each of these things exerts at the position where the robot is located
    Then, a total force is calculated by summing all those forces together with vector math
    Finally, the planner will recommend a step of fixed size, in the direction the net force pushes on.
    """
    def __init__(self):
        # Set up the list of obstacles which are present on the field always
        self.fixedObstacles:list[ForceGenerator] = []
        self.fixedObstacles.extend(FIELD_OBSTACLES_2019)
        self.fixedObstacles.extend(FIELD_LANES_2019)
        self.fixedObstacles.extend(WALLS)

        # Init the obstacle lists which go away over time
        # Right now only point obstacles supported for these
        self.transientObstcales:deque[PointObstacle] = deque(maxlen=MAX_OBSTACLES)

        # Track the goal, the distance to the goal, and the initial acceleration limiter
        self.distToGo:float = 0.0
        self.goal:Pose2d|None = None

        # Track the "lookahead" - a series of steps predicting where we will go next
        self.lookaheadTraj:list[Pose2d] = []

        #these actually aren't going to have any forces attached to them, it's just going to be for the graphics
        self.fullTransientObstacles = []
        self.thirdTransientObstacles = []
        self.almostGoneTransientObstacles = []

        # Keep things slow right when the goal changes
        self.startSlowFactor = 0.0

        #addLog("PotentialField Num Obstacles", lambda: (len(self.fixedObstacles) + len(self.transientObstcales)))
        #addLog("PotentialField Path Active", lambda: (self.goal is not None))
        #addLog("PotentialField DistToGo", lambda: self.distToGo, "m")

    def setGoal(self, nextGoal:Pose2d|None):
        """
        Sets the current goal pose of the planner. This can be changed at any time.
        Currently, A goal of None will cause the planner to not recommend any motion
        TODO: In the future, a goal of "None" might be able to imply "just move as far from obstacles as possible". 
        """
        if(nextGoal != self.goal):
            # New goal, reset the slow factor
            self.startSlowFactor = 0.0

        for lane in FIELD_LANES_2019:
            if(nextGoal == GOAL_SPEAKER):
                # Speaker/Spaceship requires lanes on opposite side to prevent local minima
                lane.strength = 0.5
            else:
                # But for other goals, don't let them impact the path
                lane.strength = 0.0

        self.goal = nextGoal

    def addObstacleObservation(self, obs:PointObstacle):
        """
        Call this to report a new, transient obstacle observed on the field.
        Right now, only point obstacles supported
        """

        alreadyKnown = False

        # First, check if there's an existing obstacle close to it
        for existingObs in self.transientObstcales:
            dist  = (obs.getTrans()[0] - existingObs.getTrans()[0]).norm()
            if (dist < 0.25):
                # We're within half a meter of an existing obstacle. Let's just assume it's the same one and 
                # move that existing one to that location.
                existingObs.strength = obs.strength
                existingObs.location = obs.location
                alreadyKnown = True
                break

        # However, if this is a new one, append it into the list
        if(not alreadyKnown):
            self.transientObstcales.append(obs)
            # Note - the deque is implemented to automatically discard from the "left" side (oldest)
            # If we grow it past the max number of tracked obstacles.

    def getGoalForce(self, curLocation:Translation2d) -> Force:
        """
        Calculate and return the force due to the goal.
        Currently, Goal force is assumed to always have the same magnititude, and always pointing toward the goal.
        """
        if(self.goal is not None):
            displacement = self.goal.translation() - curLocation
            direction = displacement/displacement.norm()
            mag = GOAL_STRENGTH * (1 + 1.0/(0.0001 + displacement.norm()))
            return Force(direction.x*mag, direction.y*mag)
        else:
            # no goal, no force
            return Force()

    def _decayObservations(self):
        """
        Transient obstacles decay in strength over time. 
        This function decays the obstacle's strength, and removes obstacles which have zero strength.
        It should be called once per periodic loop.
        """
        obsToRemove = []

        # Decay obstacles, tracking which ones to remove
        for obs in self.transientObstcales:
            obs.strength -= TRANSIENT_OBS_DECAY_PER_LOOP
            if(obs.strength <= 0.0):
                # fully decayed obstacle, remove.
                obsToRemove.append(obs)

        # Remove all fully-decayed obstacles
        for obs in obsToRemove:
            self.transientObstcales.remove(obs)

    def getObstacleStrengths(self) -> tuple[list[Translation2d], list[Translation2d], list[Translation2d], list[Translation2d]]:
        #these are all Translation 2ds, or should be, but we can't say that if we want to return all 4. 
        fixedObstacleTranslations = []
        fullTransientObstacleTranslations =[]
        thirdTransientObstacleTranslations = []
        almostGoneTransientObstacleTranslations = []
        
        # Fixed obstacles
        for x in self.fixedObstacles:
            fixedObstacleTranslations.extend(x.getTrans())

        # Transient Obstacles
        for x in self.transientObstcales:
            if x.strength > .5:
                fullTransientObstacleTranslations.extend(x.getTrans())
            elif x.strength > .33:
                thirdTransientObstacleTranslations.extend(x.getTrans())
            else:
                almostGoneTransientObstacleTranslations.extend(x.getTrans())

        return fixedObstacleTranslations, fullTransientObstacleTranslations, thirdTransientObstacleTranslations, almostGoneTransientObstacleTranslations

    def getObstacleTransList(self) -> list[Translation2d]:
        """
        Return a list of translations (x/y points), representing the current obstacles
        Just for telemetry purposes now
        """
        retArr = []
        for obstacle in self.fixedObstacles:
            retArr.extend(obstacle.getTrans())
        for obstacle in self.transientObstcales:
            retArr.extend(obstacle.getTrans())

        return retArr
    
    def isStuck(self) -> bool:
        """
        Based on current lookahead trajectory, see if we expect to make progress in the near future,
        or if we're stuck in ... basically ... the same spot. IE, at a local minima
        """
        if(len(self.lookaheadTraj) > 3 and self.goal is not None):
            start = self.lookaheadTraj[0]
            end = self.lookaheadTraj[-1]
            dist = (end-start).translation().norm()
            distToGoal = (end - self.goal).translation().norm()
            return dist < STUCK_DIST and distToGoal > LOOKAHEAD_DIST_M * 2
        else:
            return False
    
    def atGoal(self, pose:Pose2d)->bool:
        """
        Checks if we're within margin of the set goal. Defaults to True if there is no goal.
        """
        if(self.goal is None):
            return True
        else:
            err = (self.goal - pose)
            transClose = err.translation().norm() < GOAL_MARGIN_M
            rotClose = abs(err.rotation().degrees()) < GOAL_MARGIN_DEG
            return transClose and rotClose

    def _getForceAtTrans(self, trans:Translation2d)->Force:
        """
        Calculates the total force at a given X/Y point on the field.
        This is the sum of:
          1) Goal attractive force
          2) Obstacle repulsive force
          3) Lane attractive force (TODO still)
        """
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
    
    def update(self, curCmd:DrivetrainCommand, stepSize_m:float) -> DrivetrainCommand:

        # Update the initial "don't start too fast" factor
        self.startSlowFactor += 2.0 * Ts
        self.startSlowFactor = min(self.startSlowFactor, 1.0)

        nextCmd = self._getCmd(curCmd, stepSize_m)

        if(TimedRobot.isSimulation()):
            # Lookahead is for telemetry and debug only, and is very
            # time expensive. We'll do it in simulation, but not on the
            # real robot.
            self._doLookahead(curCmd)

        return nextCmd

    def _getCmd(self, curCmd:DrivetrainCommand, stepSize_m:float) -> DrivetrainCommand:
        """
        Given a starting pose, and a maximum step size to take, produce a drivetrain command which moves the robot in the best direction
        """
        retVal = DrivetrainCommand() # Default, no command
        curPose = curCmd.desPose

        if(self.goal is not None):
            curTrans = curPose.translation()
            self.distToGo = (curTrans - self.goal.translation()).norm()

            if(not self.atGoal(curPose)):
                # Only calculate a nonzero command if we have a goal and we're not near it.

                # Slow down when we're near the goal
                slowFactor = GOAL_SLOW_DOWN_MAP.lookup(self.distToGo)


                nextTrans = curTrans

                for _ in range(4):

                    if (nextTrans - curTrans).norm() >= stepSize_m:
                        break

                    netForce = self._getForceAtTrans(nextTrans)

                    # Calculate a substep in the direction of the force
                    step = Translation2d(stepSize_m*netForce.unitX()*0.5, stepSize_m*netForce.unitY()*0.5) 

                    # Take that step
                    nextTrans += step


                # Calculate the rotation command separately
                # First get the error between where we're at, and where we want to be
                rotErr = self.goal.rotation() - curPose.rotation()
                # Take a step in the direction of the error, but limit the step size by the max rotation rate
                rotStep = limit(rotErr.radians(), ROT_RAD_PER_SEC * Ts)

                # Assemble velocity commands based on the step we took
                # Note that depending on how the substeps fell, we might have taken more than a full step
                # We continue to take a step _in the direction_ of the sum of the substeps, but of
                # the correct size.

                # First, Scale total step to be unit length
                totalStep = (nextTrans - curTrans)
                totalStep = totalStep * (1.0/totalStep.norm())

                # Then, Scale totalStep to the right size
                totalStep *= (stepSize_m * slowFactor * self.startSlowFactor)

                # Periodic loops run at Ts sec/loop
                retVal.velX = totalStep.X() / Ts
                retVal.velY = totalStep.Y() / Ts
                retVal.velT = rotStep / Ts
                retVal.desPose = Pose2d(curTrans + totalStep, curPose.rotation() + Rotation2d(rotStep))


            else:
                # Case - at goal - just send the goal as the desired pose with (hopefully) no velocity
                retVal.velX = 0.0
                retVal.velY = 0.0
                retVal.velT = 0.0
                retVal.desPose = self.goal
        else:
            self.distToGo = 0.0
        
        return retVal
    
    def _doLookahead(self, curCmd:DrivetrainCommand):
        """
        Perform the lookahead operation - Plan ahead out to a maximum distance, or until we detect we are stuck
        """

        self.lookaheadTraj = []
        if(self.goal is not None):
            cc = curCmd
            self.lookaheadTraj.append(cc.desPose)

            for _ in range(0,LOOKAHEAD_STEPS):
                tmp = self._getCmd(cc, LOOKAHEAD_STEP_SIZE)
                cp = tmp.desPose
                self.lookaheadTraj.append(cc.desPose)

                if((cp - self.goal).translation().norm() < LOOKAHEAD_STEP_SIZE):
                    # At the goal, no need to keep looking ahead
                    break

    def getLookaheadTraj(self) -> list[Pose2d]:
        """
        gets the current list of lookahead poses
        """

        if(self.goal is not None):
            return self.lookaheadTraj
        else:
            return []
