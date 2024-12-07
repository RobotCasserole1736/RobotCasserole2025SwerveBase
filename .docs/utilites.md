# Utilities

A number of small utility functions are provided. 

It's possible these get dissolved at some point and show up within WPILib, at which point we can remove our verions.

## Alliance Transform

For past few seasons, the field has not been _diagonally symmetric_. This means that if you comare how the field looks while standing at the red or blue alliance walls, the main field elements will be "mirrored" left/right. A diagonally symmetric field would keep the field appearing the same, no matter which side you are on.

As long as this is the case, we have to use a _truly global_ origin for the field. By convention, we match with WPILib, and pick the corner of the blue alliance wall and the long field wall that would be to your _right_ if you standing as a driver on the blue alliance. Positive X goes "downfield" toward the red alliance, and positive Y goes to your left.

To keep our code simple, we design all goals, autonomous routines, and other logic assuming we are on the blue alliance. However, for any specific measurement or value whcih needs to be _differnt_ for being on the red alliance, we use the `transform()` function to switch the value from blue to red accordingly.

Logic which does this needs to not cache any values, as the alliance can and will change at runtime (either due to the FMS connecting and providing the correct alliance, or from the software team fiddling around and switching alliance while the robot is disabled).

## Calibration

When writing robot code, a frequent issue is having a value which is _usually_ constant during runtime, but might need to be tuned or tweaked by the software team. 

PID constants, debounce times, discrete setpoints, and rate limits are common examples of this.

Critically, the deploy process is not _super_ fast - if a value needs to be iterated quickly, it makes sense to write a bit of code to allow it to be cahnged on the fly, without redeploying.

The `Calibration` object helps do this. Users should declare `Calibration` objects with the name and default value. Optionally, units, min, and max values can be provided too.

At runtime, in code, use the `.get()` method to get the current value of the Calibration.

Additionally, an `.isChanged()` API is present to check if the calibration changed since the last time `.get()` was called - this allows the user to trigger one-time operations when the cal cahnges (for example, sending CAN bus messages to a motor controller to change its current limit).

Once declared, a calibration object should show up on the website as availble for calibration.

### Behind the Scenes

When a `Calibration` is declared, it registers itself with the `CalibrationWrangler`. The `CalibrationWrangelr` is a singleton that the web server uses to get the full list of declared calibrations, present them to the user via network tables, and update their values when they changed.

## Crash Logging

Python code has the ability to crash at runtime on the robot. This causes the code on the robot to stop running, restart from scratch, and re-enable (if in match). This process takes a few minutes and, in general, is not desireable.

When this happens on the field, it's useful to generate records of what happened to help the software team debug after the match.

The `CrashLogger`'s job is to facilitate that. Additionally, it provides a way for users to inject their own messages at runtime.

When initialized, the `CrashLogger` sets up a python `logging.FileHandler()` to recieve log messages. 

Periodically, the `update()` function checks that the FMS is connected and has provided certain match-specific pieces of information. Once these are available, they are also (once) written to log.

Becuase our file logger is attached to the main logger, python itself facilitates making sure any crash information is piped into `stderr`, which in turn ends up in the log.

## External Drive Management

Casserole has semi-standardized on using an external USB drive to store our runtime time-series logs. This helps prevent the main hard drive (flash rom) of the RIO from filling up from log files.

However, it's not guarnteed we rememberd to plug in a drive, and we'd like our code not to crash in this case.

The `ExtDriveManager` takes care of finding the right path to log to (which is slightly differnet in simulation and real robot), checking the path exists and is writable, making directories as needed, and indicating drive health with a Fault. If the log is not avaialble at startup, crash logging and data logging is disabled.

## Faults

A "Fault" is an abnoral condition on the robot which should prevent the robot from fully functioning. There are often things we can detect in software which we can alert the pit crew to.

The `Fault` object can be declared, with a message to be shown to the user when it is active. Faults start inactive by default.

The `.set()`, `.setFaulted()`, and `.setNoFault()` methods shoudl be called to alter whether the fault is anunciating "active" to the user or not.

Caution should be applied in triggering a fault to be active without something to fix. We want to drive the behavior that when the fault indicator starts blinking, pit crew treats the situation as a "stop to fix" scenario. Having faults that are "supposed to be active" will hide new, true faults that need fixing.

### Behind the Scenes - Fault Wrangler

The `FaultWrangler` is a singleton that all `Fault`'s register themselves with on init. 

On `update`, the fault wrangler checks for any active faults. If 1 o rmore faults are active:

1. The NT entry `faultDescription` cycles through all the strings of the active faults
2. A red LED begins blinking on the robot.

Faults are displayed in a few different ways:

### Driver Dashboard

An Icon/String widget can be used to display the current `faultDescription` string, and blink if the fault Count is greater than 1

### Fault LED

A roboRIO output is used to pulse an LED on and off when one or more faults are active. This is a signal to the pit crew to stop and fix the robot.

### Heartbeat LED

If code is completely frozen, faults will not annunicate. To help make sure code is running, in all conditions, a white led is pulsed on and off slowly. Pit crew should know not to expect normal robot function until this LED is pulsing.

## Map Lookup 2D

A common way to define functions in embedded software is to _sample_ the function's output at various input values. Then, these are used to "map" future inputs to outputs by linearlly interpolating between the known points, at the given input.

We use the `MapLookup2D` object to do this operation. 

It is declared with a set of `(input, output)` tuples that represent the points in the map.

The first and last outputs are "sampled and held" for all input values above or below the first and last input values.

For example, y = 2x might be defined as:

```py
doublerMap = MapLookup2d([
    (-100,-200),
    (0,0),
    (100,200),
])
```

THe `.lookup(xval)` method is used to look up an output value for the given `xval` input.

## Power & RIO Monitors

The `PowerMonitor` has largely been stripped for now.

In the future, it's important to make sure that at least battery voltage and current are logged. Per-motor current draw might also be useful to log, if resources exist to do it.

The `RIOMonitor` class has also been failry stripped out. It's job was to monitor specific roboRIO specific things that coudl be useful for debugging issues in the pit afterward. For example:

1. RAM usage
2. CPU Load
3. CAN bus load, CAN bus error count
4. Disk usage
5. input voltage
6. voltage rail faults.

In the future we'll want to expand these back out. However, for now, theire funcitonality has been stripped down to prevent high CPU loads.

## Segment Time Tracker

To keep our code running well, our code needs to run fast (at least fast enough to let it execute all periodic actions once every 20ms).

Tracking how fast our code executes is an important component of keeping it running well.

The `SegmentTimeTracker` class can help with this. After init'ing it, call the following:

`start()` at the start of a periodic loop
`end()` at the end of a periodic loop

These will help produce metrics in the dashboard about how long our code spent running, and what period it was _actually_ called at.

Additionally, `mark(thing)` can be called after certain actions to indiciate that `thing` just finished. If the code starts running slow, a report will occastionally be dumped out to indicate a list of how long each `thing` that was marked took to run.

## Singleton Infrastructure

Singletons are a programming concept for creating a class which has only one instance, and can be accessed anywhere.

This is not super common out in non-robotics programming, but ends up being a pretty useful concept when we create classes designed around a thing on our robot that we can be certain there is only ever one of (drivetrain? arm?). 

Declaring a class as inheriting from `Singleton` causes invocation of `MySingletonClass()` constructor to return the same instance of the class.

Note taht for _unit testing purposes specifically_, we need to make sure each class's destructor is called. We do this by calling `destroyAllSingletonInstances` at the end of each testcase to reset the simulation back to "no code running" state.
