# Picking a Drivtrain Control Strategy

There are multiple parts of robot code which might want to control the drivetrain. The following logic is used to determine what is actually controlling it.

## Arbitration

The drivetrain logic uses a "chained" approach to arbitration. Each feature takes in the command from the "upstream" features, has an *opportunity* to modify any part of the command, and passes the result downstream. 

As more automatic assist or control features are added, they should be put into this stack. The further "downstream" in the list they are, the higher priority the feature will have.

### Manual Control

Manual control takes velocity commands from the driver's joysticks. There is no specific pose desired, only velocity is commanded.

Manual control must always be available. It is the default option if no other feature is taking control.

### Autonomous Path Following

During autonomous, pre-planned paths are often executed. At each time step, a desired pose and velocity is read out of a file and passed to the drivetrain. This only happens if the autonomous path planning logic is active.

### Teleop Navigation

The navigation stack can create commands to automatically drive the robot toward a goal, avoiding obstacles. This generates velocity commands, and a desired pose.

### TODO - align to gamepices

This logic should create rotational commands which point the robot at a gamepiece, but otherwise do not alter the X/Y velocity commands.

This has not yet been written

