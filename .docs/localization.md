# Localization

Localization is the process of estimating where, on the field, the robot is at.

This "where" is answered by three numbers:

X position - in meters - from the origin
Y position - in meters - from the origin
Rotation - in radians - in angular deflection from the positive X axis, toward the positive Y axis.

Collectively, these are known as a "Pose" - specifically, a Pose2d

The sensors on our robot provide clues as to these numbers, but always some inaccuracy. Theses clues provide overlapping information, which we must *fuse* into a single estimate.

We use WPILib's *Kalman Filter* to fuse these pieces of information together, accounting for the fact each has some inaccuracy.

## About Inaccuracy

All sensors measure a particular quantity, and add some amount of random *noise* to that signal. The noise is in the same units as the quantity being measured.

When looking at sensor data, you can often see the signal "jittering" around the real value. This noise can be at least roughly measured.

The most common way to understand this noise is to assume it is random, with a *Gaussian Distribution*. The amount of noise will be proportional to the *standard deviation* of the noise's distribution.

The Kalman Filter takes the standard deviation of each measurement as an input, as a way to know how much to "trust" the measurement. Measurements with high standard deviations have a lot of noise, and are not particularly trustworthy.

Trustworthy measurements will change the the estimate of the robot's Pose rapidly. Untrustworthy measurements will take a lot of time to have an impact on the Pose estimate.

## Data Sources

### Gyroscope

FRC robots almost always include a gyroscope, which measures rotational velocity. By adding up the rotational velocity measurements over time, the sensor measures changes in the robot's angular position.

The gyroscope is one of the most accurate, least-noisy measurements of robot position. It should only drift by a degree or two every minute. However, it only measures one component of pose. Additionally, it is at best relative to a given starting pose, which must be accurate.

### Swerve Module Encoders

The encoders inside the swerve modules (wheel motors, and absolute encoders measuring azimuth position) provide a good estimate of movement. As long as wheels are rolling along the ground (and not slipping), linear displacement can be derived through determining distance rolled from number of rotations, multiplied by the circumference of the wheel.

The wheel encoders are also generally very accurate, with most noise being driven by slippage during high-acceleration maneuvers. Additionally, it is at best relative to a given starting pose, which must be accurate.


### AprilTags

Using a camera and a coprocessor, we can estimate our pose relative to the *fiducial markers* that FIRST provides.

These estimates provide all three components of Pose, and are absolute - they do not care whether the initial pose estimate was accurate or not. However, the signal often has a lot of latency (as it takes 100ms or more to get the image, process it, and send the result to the roboRIO). Additionally, their accuracy varies, depending on how far away the observed tag is, and whether or not [the observed pose is impacted by a common optical illusion.](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html#d-to-3d-ambiguity).

## Initial Position

Software must make an accurate initial assumption of pose. This is done through one of two ways:

### Autonomous Init

Each autonomous routine must provide an assumed starting pose, where the drive team places the robot at the start of the routine.

This pose is returned from the sequencer, and used by the pose estimation logic to reset pose at the start of the match.

### Teleop Gyro Reset

The code support resetting the rotational component of pose to be aligned toward "downfield" - this helps during development or driver practice where autonomous is not run.

This only fixes the rotational component of pose. While rotational position is most important, X/Y translation can only be corrected by looking at an apriltag.