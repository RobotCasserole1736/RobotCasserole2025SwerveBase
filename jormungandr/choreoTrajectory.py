from __future__ import annotations
import math

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from utils.constants import FIELD_X_M

def _floatInterp(start, end, frac) -> float:
    """
    Note, in theory wpilib should have this, but the python version in 2024
    didn't seem to match up with what java/c++ had added - not sure why it
    was missing. It was simple enough we just replicated it here though.
    """
    return start + (end - start) * frac


class ChoreoTrajectoryState:
    """
    Represents what state the robot should be in at a particular time. 
    State includes both position and velocity in all dimensions
    """
    def __init__(
        self,
        timestamp: float,
        x: float,
        y: float,
        heading: float,
        velocityX: float,
        velocityY: float,
        angularVelocity: float,
    ):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.heading = heading
        self.velocityX = velocityX
        self.velocityY = velocityY
        self.angularVelocity = angularVelocity

    def getPose(self) -> Pose2d:
        """
        Get a pose object associated with this state
        """
        return Pose2d(self.x, self.y, Rotation2d(value=self.heading))

    def getChassisSpeeds(self) -> ChassisSpeeds:
        """
        Get a chassis speed associated with this object
        """
        return ChassisSpeeds(self.velocityX, self.velocityY, self.angularVelocity)

    def asArray(self) -> list[float]:
        """
        Dump all values into an array in a specific order. This is mostly 
        for telemetry purposes.
        """
        return [
            self.timestamp,
            self.x,
            self.y,
            self.heading,
            self.velocityX,
            self.velocityY,
            self.angularVelocity,
        ]

    def interpolate(
        self, endValue: ChoreoTrajectoryState, t: float
    ) -> ChoreoTrajectoryState:
        """
        Perform Linear Interpolation between this trajectory state, and some other one.
        Useful for getting a state that's part way between two defined states.
        """
        scale = (t - self.timestamp) / (endValue.timestamp - self.timestamp)

        return ChoreoTrajectoryState(
            t,
            _floatInterp(self.x, endValue.x, scale),
            _floatInterp(self.y, endValue.y, scale),
            _floatInterp(self.heading, endValue.heading, scale),
            _floatInterp(self.velocityX, endValue.velocityX, scale),
            _floatInterp(self.velocityY, endValue.velocityY, scale),
            _floatInterp(self.angularVelocity, endValue.angularVelocity, scale),
        )

    def flipped(self):
        """
        Mirror the trajectory state to the other alliance. Note that this assumes the style of 
        symmetry that the 2023/2024 fields had. Additionally, note that this is kinda the same thing
        that we did in allianceTransformUtils, but the Choreo library also provided the functionality.
        It's here too. Depending on how Choreo architects in 2025+, we might want to move this.
        """
        return ChoreoTrajectoryState(
            self.timestamp,
            FIELD_X_M - self.x,
            self.y,
            math.pi - self.heading,
            self.velocityX * -1.0,
            self.velocityY,
            self.angularVelocity * -1.0,
        )


class ChoreoTrajectory:
    """
    A trajectory is a series of states that describes how the robot moves through
    a pre-defined path on the field. Choreo is a tool that generates "optimal" paths
    based on knowledge of what a swerve drive can physically do. These are designed
    and generated on a laptop. Then, before autonomous, the robot loads the 
    trajectory and sends commands to the drivetrain based on the trajectory
    """
    def __init__(self, samples: list[ChoreoTrajectoryState]):
        self.samples = samples

    def _sampleImpl(self, timestamp) -> ChoreoTrajectoryState:
        """
        Actual implementation of getting the robot state at a specific timestamp
        """

        # Handle timestamps outside the trajectory range
        if timestamp < self.samples[0].timestamp:
            return self.samples[0]
        if timestamp > self.getTotalTime():
            return self.samples[-1]


        # Binary search to find the two states on either side of the requested timestamps
        # TODO - We might be able to pick better low/high guesses. We know we'll almost always
        # be iterating throught the trajectory in roughly 20ms steps. It would be helpful to
        # not have to search the full array each time.
        low = 0
        high = len(self.samples) - 1
        while low != high:
            mid = math.floor((low + high) / 2)
            if self.samples[mid].timestamp < timestamp:
                low = mid + 1
            else:
                high = mid

        # Handle case near start of trajectory
        if low == 0:
            return self.samples[0]

        # Find the states on either side of the requested time
        behindState = self.samples[low - 1]
        aheadState = self.samples[low]

        if aheadState.timestamp - behindState.timestamp < 1e-6:
            # meh states are so close, just give back one of them
            return aheadState

        # Perform the actual interpolation
        return behindState.interpolate(aheadState, timestamp)

    def sample(
        self, timestamp: float, mirrorForRedAlliance: bool = False
    ) -> ChoreoTrajectoryState:
        """
        Return the desired state of the robot at the specified time 
        """
        tmp = self._sampleImpl(timestamp)

        # Include flipping for alliance if needed.
        if mirrorForRedAlliance:
            return tmp.flipped()
        else:
            # no mirroring
            return tmp

    def getInitialPose(self) -> Pose2d:
        """
        Returns the pose we assume the robot starts the trajectory at.
        """
        return self.samples[0].getPose()

    def getFinalPose(self) -> Pose2d:
        """
        Returns the pose we hope the robot will end the trajectory at
        """
        return self.samples[-1].getPose()

    def getTotalTime(self) -> float:
        """
        Return the duration of the trajectory in seconds
        """
        return self.samples[-1].timestamp

    def getPoses(self) -> list[Pose2d]:
        """
        Return the list of all poses associated with this trajectory. Mostly useful for telemetry.
        """
        return [x.getPose() for x in self.samples]

    def flipped(self) -> ChoreoTrajectory:
        """
        Return the whole trajectory flipped to the opposite alliance.
        """
        return ChoreoTrajectory([x.flipped() for x in self.samples])
