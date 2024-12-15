# vsCode Project Setup

A few things were added to our vsCode project to try to make vsCode feel closer to Java's workflow.

## Setup

Configuration is saved in `.json` files saved in the `.vscode` folder.

## Taks - `task.json`

`tasks.json` creates menu items for the common commands we run from robotpy. These include:

. Running the Test Suite in Simulation
. Deploying code to the RIO (normal)
. Viewing netconsole (the stdout forwarder that you see in Driver Station)
. Deploying the code to the RIO (debug mode)

## Debug Configurations - `launch.json`

### Simulation

This invokes the robotpy command to do simulation, but invokes it within a python debugger context.

### On-RIO

This first re-deploys the code with special flags to enable debug, waits for the code to start up, then connects to the RIO remotely (which _should_ be running a debug server at that point)

There is a small bit required inside of robot code to support debugging. In `roboty.py` - the function `remoteRIODebugSupport()` should be invoked early on in execution. This function shall:

1. Detect that we're running robot code on a RIO, with debug mode active.
2. Import the `debugpy` module JIT (it's not needed otherwise and was causing some sim problems).
3. Enable remote debug connections from any client IP on port 5678
4. Wait indefinitely for a client to connect

Step 4 is critical to be sure that a breakpoint set early in initlization is not missed. This was funcitonality specifically not present in robotpy in 2024. 

This code is fairly intrusive to robot behavior, and overlaps debug support that robotpy was attempting to add in 2024. It needs review in 2025 to see if it's still needed.

### Test

This invokes the robotpy command to run the test suite, but invokes it within a python debugger context
