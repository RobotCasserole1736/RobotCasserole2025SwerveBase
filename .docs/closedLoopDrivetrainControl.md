# Closed Loop Drivetrain Control

We achieve smooth control of our drivetrain through a slightly modified [Holonomnic Drivetrain Controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html)

Swerve Drive has three _independent_ axes of control - Translation (X, Y), and Rotation (Theta). Unlke a tank drive, where side-to-side translation can not be done independently of forward motion, our control startegy may be easier.

Our control logic feeds the drivetrain with desired X, Y, and Rotational Velocity as the sum of two sources:

1) Open Loop velocity command
2) Closed loop position-correction command

All control starategies (manual, auto-drive, trajectory, etc.) must supply an open loop velocity command for all three axes at all times. Manual control gets these from the joysticks, auto-drive gets them from looking at how we're moving from the current to the next step, trajectories in autonomous pre-calculate the velocity throuhgout the trajectory.

Optionally, a `Pose2d` might be available from the controlling source. Manual control generally will not have this, but auto-drive and trajectory control probably will.

Since we have `odometry` to calculate an estimated `Pose2d` of our robot, we can subtract this from our commanded `Pose2d` to get an error in position. Each axis of the error (3 in total) are run through an independent PID controller, which in turn generates a small additional _corrective_ velocity to get the robot's estiamted pose closer to where we want to be.

Notably, in the past, wpilib's implementation did not have open-loop control of rotation. This was a strage omission. For this reason, we've had our own minimal `HolonomicDriveController` class for quite some time.