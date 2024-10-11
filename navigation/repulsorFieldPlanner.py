from wpimath.geometry import Pose2d, Translation2d, Rotation2d

from utils.mapLookup2d import MapLookup2D
from utils.signalLogging import log

from drivetrain.drivetrainCommand import DrivetrainCommand

from navigation.navForce import Force
from navigation.obstacles import HorizontalObstacle, Obstacle, PointObstacle, VerticalObstacle
from navigation.navConstants import FIELD_X_M, FIELD_Y_M

# Relative strength of how hard the goal pulls the robot toward it
# Too big and the robot will be pulled through obstacles
# Too small and the robot will get stuck on obstacles ("local minima")
GOAL_STRENGTH = 0.04

# Maximum number of obstacles we will remember. Older obstacles are automatically removed from our memory
MAX_OBSTACLES = 20

# Rate at which transient obstacles decay in strength
# Bigger numbers here make transient obstacles disappear faster
TRANSIENT_OBS_DECAY_PER_LOOP = 0.1

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
    PointObstacle(location=Translation2d(11.0, 5.35)),
]

# Fixed Obstacles - Outer walls of the field 
B_WALL = HorizontalObstacle(y=0.0)
T_WALL = HorizontalObstacle(y=FIELD_Y_M)
T_WALL.setForceInverted(True)

L_WALL = VerticalObstacle(x=0.0)
R_WALL = VerticalObstacle(x=FIELD_X_M)
R_WALL.setForceInverted(True)

WALLS = [B_WALL, T_WALL, L_WALL, R_WALL]

# Happy Constants for the goal poses we may want to drive to
GOAL_PICKUP = Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0))
GOAL_SPEAKER = Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(180.0))

# Usually, the path planner assumes we traverse the path at a fixed velocity
# However, near the goal, we'd like to slow down. This map defines how we ramp down
# the step-size toward zero as we get closer to the goal. Once we are close enough,
# we stop taking steps and simply say the desired position is at the goal.
GOAL_MARGIN_M = 0.05
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
        self.fixedObstacles:list[Obstacle] = []
        self.fixedObstacles.extend(FIELD_OBSTACLES_2024)
        self.fixedObstacles.extend(WALLS)
        self.transientObstcales:list[Obstacle] = []
        self.distToGo:float = 0.0
        self.goal:Pose2d|None = None
        self.lookaheadTraj:list[Pose2d] = []

    def setGoal(self, goal:Pose2d|None):
        """
        Sets the current goal pose of the planner. This can be changed at any time.
        Currently, A goal of None will cause the planner to not recommend any motion
        TODO: In the future, a goal of "None" might be able to imply "just move as far from obstacles as possible". 
        """
        self.goal = goal

    def add_obstcale_observaton(self, obs:Obstacle):
        """
        Call this to report a new, transient obstacle observed on the field.
        """
        self.transientObstcales.append(obs)
        while len(self.transientObstcales) > MAX_OBSTACLES:
            self.transientObstcales.pop(0)

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

    def _decay_observations(self):
        """
        Transient obstacles decay in strength over time. 
        This function decays the obstacle's strength, and removes obstacles which have zero strength.
        It should be called once per periodic loop.
        """
        for obs in self.transientObstcales:
            obs.strength -= TRANSIENT_OBS_DECAY_PER_LOOP

        # Only keep obstacles with positive strength
        # Fully decayed obstacles have zero or negative strength.
        self.transientObstcales = [x for x in self.transientObstcales if x.strength > 0.0]

    def getObstacleTransList(self) -> list[Translation2d]:
        """
        Return a list of translations (x/y points), representing the current obstacles
        Just for telemetry purposes now
        
        TODO: This really only works well for point obstacles now. 
        We'll need to consider in the future how to do this for linear obstacles
        Perhaps a list of translation pairs (start/end) ?
        """
        retArr = []
        for obstacle in self.fixedObstacles:
            retArr.extend(obstacle.getTelemTrans())
        for obstacle in self.transientObstcales:
            retArr.extend(obstacle.getTelemTrans())

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
    
    def atGoal(self, trans:Translation2d)->bool:
        """
        Checks if we're within margin of the set goal. Defaults to True if there is no goal.
        """
        if(self.goal is None):
            return True
        else:
            return (self.goal.translation() - trans).norm() < GOAL_MARGIN_M

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
    
    def update(self, curPose:Pose2d, stepSize_m:float) -> DrivetrainCommand:
        curCmd = self._getCmd(curPose, stepSize_m)
        self._doLookahead(curPose)
        return curCmd

    def _getCmd(self, curPose:Pose2d, stepSize_m:float) -> DrivetrainCommand:
        """
        Given a starting pose, and a maximum step size to take, produce a drivetrain command which moves the robot in the best direction
        """
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

                    netForce = self._getForceAtTrans(nextTrans)

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
    
    def _doLookahead(self, curPose):
        """
        Perform the lookahead operation - Plan ahead out to a maximum distance, or until we detect we are stuck
        """
        self.lookaheadTraj = []
        if(self.goal is not None):
            cp = curPose
            self.lookaheadTraj.append(cp)

            for _ in range(0,LOOKAHEAD_STEPS):
                tmp = self._getCmd(cp, LOOKAHEAD_STEP_SIZE)
                if(tmp.desPose is not None):
                    cp = tmp.desPose
                    self.lookaheadTraj.append(cp)

                    if((cp - self.goal).translation().norm() < LOOKAHEAD_STEP_SIZE):
                        # At the goal, no need to keep looking ahead
                        break
                else:
                    # Weird, no pose given back, give up.
                    break

    def updateTelemetry(self) -> list[Pose2d]:
        """
        Perform all telemetry-related tasks.
        This includes logging data
        Returns the list of Pose2D's that are the result of the lookahead operation
        """
        log("PotentialField Num Obstacles", len(self.fixedObstacles) + len(self.transientObstcales))
        log("PotentialField Path Active", self.goal is not None)
        log("PotentialField DistToGo", self.distToGo, "m")
        if(self.goal is not None):
            return self.lookaheadTraj
        else:
            return []
