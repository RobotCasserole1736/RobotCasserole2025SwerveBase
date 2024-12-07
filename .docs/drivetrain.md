# Drivetrain Control

## Swerve Drive Overview

A "swerve drive" is the most common type of drivetrain in the FIRST Robotics Competition in the last few years.

It consists of four "modules", each mounted on a corner of the robot.

Each module has two motors - one to point the wheel in a specific direction ("azimuth"), and one to actually spin the wheel ("wheel").

As a coordinate frame reminder: for our rectangular chassis, the robot's origin is defined as lying in the same plane as the floor, and in the exact center of the rectangle of the drivetrain. Positive X points toward the front of the robot, positive Y points toward the robot's left side. Positive rotation swings from +X to +Y axes.

## Overall Drivetrain Control

Our control is "field-oreiented" - the commands into the drivetrain are assumed to be velocity commands aligned with the field walls.

A few steps are needed in the overall control algorithm.

1. Rotate the incoming velocity commands to be relative to the robot's heading, not the field's +X axis. WPILib provides these functions.
2. Figure out the velocity and azimuth angle _at each module's contact patch with the ground_, using kinematics and the drivetrain dimensions. WPILib provides these functions.
3. Perform per-module state optimization (see below)
4. Send specific wheel velocity and azimuth angle commands to each module.

### Module State Optimization

At any given time, there are four possible ways a module could go from its present azimuth angle to a new commanded one:

. Rotate left, keep wheel velocity the same
. Rotate right, keep wheel velocity the same
. Rotate left, invert wheel velocity
. Rotate right, invert wheel velocity

In this was, the maximum number of degrees a module should ever have to be commanded to rotate through is 90. By optimizing the state, we reduce the amount of time the module is in a "skidding" state, where the wheel is not smoothly rolling across the floor.

WPILib provides the functions to do this, we simply have to call them.

## Module Control

Controlling the module requires controlling both motors.

### Wheel

The wheel velocity is achieved through a basic feed-forward, plus a small feedback portion.

The feed-forward model is the standard motor veloicty model, consisting of:

`kS` - static friction - maximum voltage that can be applied to the motor without motion occurring.
`kV` - number of volts to achieve a certain rotational velocity

Future adds include `kA` - number of volts to achieve a certain _change_ in rotational velocity.

Feedforward should be doing the vast majority of the control effort. Feedback is just a small additional factor to help compensate.

### Azimuth

For now, we are just using a simple P/D feedback controller on the azimuth angle. This seems to be sufficent.

Future adds could be to motion profile the commanded position of Azimuth angle, and then using that for a velocity feed-forward.

## Constants & Configuration

As with most good code, striving to minimize "magic constants" throughout the code is perferred.

Most drivetrain related constants are in `drivetrainPhysical.py`. Key ones that users may have to adjust:

. `WHEEL_BASE_HALF_*_M` - Distance from the origin of the robot, out to the center of where the wheel makes contact with the ground. Note this is not to the frame rail or the bumper. Note it's only _half_ the distance between two wheels.
. `WHEEL_GEAR_RATIO` - Reduction ratio in the modules from the driving motord down to the wheel.
. `WHEEL_RADIUS_IN` - radius of the wheel from center of rotation, to where it contacts the carpet
. `MAX_DT_MOTOR_SPEED_RPS` - maximum achievable speed from the drive motor. WPILib has most of these internally in their physical plant model configurations. 
. `*_ENCODER_MOUNT_OFFSET_RAD` - adjusts for the physical mounting offset of the module/magnet and angle sensor on each drivetrain azimuth. 
. `ROBOT_TO_*_CAM` - 3d Transforms from robot origin to the camera described (including both translation, and angle)

Other constants are present, but they are tied to things that don't change much year to year (robot weight, azimuth steer gear ratio, bumper thickness, etc.)

Most other constants in the file are derived from these constants.

### Encoder Mount Offset Cal Procedure

Must be updated whenever the module is reassembled

1. Put the robot up on blocks.
2. Reset all these values to 0, deploy code
3. Pull up dashboard with encoder readings (in radians)
4. Using a square, twist the modules by hand until they are aligned with the robot's chassis
5. Read out the encoder readings for each module, put them here
6. Redeploy code, verify that the  encoder readings are correct as each module is manually rotated

