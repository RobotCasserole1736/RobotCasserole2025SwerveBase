# Navigation

The navigation stack is a new-for-2025 development the team made to attempt to add flexible, automatic, on-demand driving to teleop.

The driving is aware of obstacles, both those assumed always-present on the field, and one dynamically detected with cameras.

## General Principals

The underlying algorithim is called "Repulsor Fields" or "Potential Fields". See [this Columbia University Lecture](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf) for a brief introduction.

The planner maintains a list of all obstacles on the field, as well as a goal position.

Every loop, the robot's last desired pose is used to calculate the force each obstacle or goal has on the robot.

The sum of these forces produces a vector in a specific direction on the field. The robot path plans one step into that direction. This process is repeated at every loop.

### Advantages of this Method

Compared to other solutions, this method...

1. Is relatively computationally light. It spreads the planning effort over many control loops, only calculating next steps as needed.
2. Does not require complex logic to replan in the background when obstacles change. 

### Disadvantages of this Method

1. It gets stuck at local minima - Other methods can be more aggressive about looking at the whole map, and providing a reasonable path when "boxed-in" by obstacles
2. Manual Tuning - obstacle/goal force strengths are arbitrary values that were tuned "to taste" in simulation to make paths look nice, and not generate local minima.

## Force Generators

Our codebase defines a number of objects which exert force on a given position on the field. 

### Force Shape

All force generating objects use the [*logistic function*](https://en.wikipedia.org/wiki/Logistic_function). 

Large distances (far away from the obstacle) correspond to large negative values input to the function. Small distances (very close to the obstacle) should produce large positive values.

While this is not accurate to how forces like gravity work in the real world, it has useful properties for tuning the behavior of the field:

1. Far away from the obstacle (large negative input values to the function), the strength goes to zero.
2. The X=0 point is always "half strength"
3. By shifting the X axis left/right, the "radius" of effect of the obstacle can be impacted.
4. By stretching or shrinking the Y axis, the strength of the force can be changed
5. By stretching or shrinking the X axis, the "steepness" of the transition from no force to lots of force can be tuned.
6. The function does not go to infinity at any point.

However, this large number of tuning parameters adds to the complexity of getting a good overall force field, with no local minima.

### PointObstacle

Generates a force that radiates outward from the center point, and whose strength is determined by the logistic function of the distance from that point.

They work well for modeling roughly-circular obstacles (like robots, or posts).

### HorizontalObstacle

Generates a force that radiates toward the positive or negative Y axis. The strength of the force is determined by the logistic function of the distance from a given Y coordinate.

It is useful for keeping the robot away from the longer walls on the field (left or right walls, as viewed from the driver station).

### VerticalObstacle

Generates a force that radiates toward the positive or negative X axis. The strength of the force is determined by the logistic function of the distance from a given X coordinate.

It is useful for keeping the robot away from the shorter walls on the field (red or blue alliance walls).

### Wall

Generates a force that radiates outward, perpendicular to a line segment (defined between two points). 

It is useful keeping the robot away from a wall or ramp in the middle of the field.

Note that there is redundancy in behavior between HorizontalObstacle, VerticalObstacle, Wall types. In the future, this should be removed.

### Lane

Generates a force that pushes the robot toward and along a line segment (defined between two points).

It is useful for marking certain areas of the field as "desireable" to travel through, especially if the default strategy of "go toward the goal" is insufficient. 


### Goal (TODO)

This class has not yet been added. The existing functionality is to apply a constant force in the direction of the goal itself. This should be moved into a more well-defined class.


## Repulsor Field Planner

### Transient Obstacles

In general, transient obstacles decay in strength whenever they are not observed, eventually being removed once they have zero strength.

Whenever an obstacle is observed by a camera, the existing list of obstacles is searched to find if any obstacles are close to the new observation. If so, the existing obstacle is reset to max strength, and moved into the observed position. If no obstacle is close enough, a new obstacle is added to the list.

There is a maximum number of observed obstacles - if more than this max are observed, the oldest obstacle is removed from the list.

Fixed obstacles must be tracked in a separate list, as they do not decay and are constant.

### Path Traversal Speed

Two "slowdown factors" are set up:

1) Start Slow Factor - ramps from 0.0 up to 1.0 after the goal changes. This makes sure the commanded position slowly accelerates from a stop, providing a more realistic change in velocity.
2) End Slow Factor - as the commanded position gets closer to the goal, the commanded speed reduces to slow the robot down as it approaches the goal.

Other than these two factors, the step size will be set to the maximum desired speed of the path planner. This is usually some fraction (~85%?) of the maximum speed of the drivetrain. The bigger this fraction, the less margin the drivetrain has to correct for disturbances.

### Next-Pose Solver

In general, the basic algorithm for going from the current commanded pose to the next one is:

1) Calculate the direction the force on the current commanded pose points in
2) Take a step of size (max_vel) * (loop time) * (slow_factors) in that direction

However, when the force changes rapidly from one pose to another, instability in the path can result, as the pose bounces back-and-forth across the sharp change in force.

As Larry indicates: the main way to solve this is to take smaller steps.

Right now, we are configured to take smaller "substeps" (half the size of a normal step) in a short loop, until the next pose is at least one full step away from the current commanded pose. Care is taken to rescale this step to the correct length, so the correct velocity is commanded. 

The number of subsets must be limited, to prevent an infinite loop in the case of a local minima.

### Stuck Detection

TODO.

Should be looking at a short history of commanded positions, and determining whether "forward progress" is being made. If the last couple points are at similar spots, declare "stuck".

### Lookahead

This functionality is only active in sim, as it is computationally intensive and (so far) only useful for telemetry and testing.

The lookahead operation plans a number of steps ahead of the position, to help visualize what the pathplanner's behavior will be. These poses are sent to the telemetry logic to display, but are otherwise discarded.