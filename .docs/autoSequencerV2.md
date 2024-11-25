# AutoSequencer V2

## Goals
Better flexibility for expressing complex sequences of autonomous routines

## Key Updates from V1
Rename `Event` to `Command` for better alignment with wpilib command based

Introduce `CommandGroup` as an ordered list of `Command`s to be run together

Introduce the following flavors of groups:
* `CommandRaceGroup` 
  * member commands run at the same time, finishing when the FIRST one is done
  * Extends `Command`, with pass-trough on all functions to do the same thing on all commands in the group, except for `isDone()` implemented as an OR
* `CommandParallelGroup` 
  * member commands run at the same time, finishing when ALL are done
  * Extends `Command`, with pass-trough on all functions to do the same thing on all commands in the group, except for `isDone()` implemented as an AND
* `CommandSequentialGroup`
  * member commands run one after another

Requirements for `Command`
* Abstract (extender-must-implement) methods for:
  * `initialize` - called once right before the first call to `execute`
  * `execute` - Called at a rate as long as the command is active
  * `end(boolean interrupted)` - called once right after the final call to `execute`. `interrupted` indicates whether the end was due to this command finishing "naturally" or because somethign else stopped it prematurely
  * `isDone()` - return true if we're done, false otherwise
* Commands also implement convienence "composition" methods:
  * `withTimeout` - returns a `raceWith()` a `WaitCommand`
  * `raceWith()` - returns a race group with this command the input commands
  * `alongWith()` - returns a parallel group with this command the input commands
  * `andThen()` - returns a sequential group with this command and the input command
* Commands can `schedule()` or `cancel()` themselves with the `AutoSequencer()`

Pre-supplied implementations of `Command`:
* `WaitCommand` - waits the given duration

Existing requirements for `Mode`
* Singular `CommandSequentialGroup` for all `Command`'s in the mode
* Must provide API to:
  * Supply the initial pose 

Existing requirements for `AutoSequencer`:
* Top-level state machine for a command group
* Singleton
* Registration of multiple auto `Modes`, including publishing the available list to NT
* NT-based selection of the current auto event
* Ticks the `Mode`'s `CommandSequentialGroup` forward

