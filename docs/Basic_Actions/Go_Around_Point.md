
# Basic action: Go Around the Point

##### [Go back to main page](../../Documentation.md)


### Go Around the Point
This action generates a robot control signal (wheel velocities) to go around a point at a given distance from the robot's centre 
(distance perpendicular to the robot's heading).

Possible use cases are, for example, getting into an appropriate position to kick the ball and obstacle avoidance.

This action is implemented in the function `go_around_the_point()`. 

## Assumptions, Inputs and Outputs
__Assumptions__: The robot's model is known

__Inputs__: Radius of Curvature of the path, direction: 1 = clockwise, -1 = counterclockwise.

__Output__: Robot control command (single instance)

## Logic
In the two-wheel differential drive system the radius of curvature (R), the distance between wheels (l) and the ratio of velocities of the left (Vl) and the
right (Vr) wheels are related. The relation is given by the following equation:

`Vl/Vr = (R - l/2)/(R + l/2)`.

Finally, we check the maximum velocity. If the velocity of any wheel exceeds a threshold value, a new calculation is done.

<p align="center">
  <img src="../../Images/GoingAroundOne.jpeg">
</p>

__Figure 1__:The Robot and the point it has to go around.

<p align="center">
  <img src="../../Images/GoAroundAPoint.png">
</p>

__Figure 2__:</b> The derivation of the formula using simalarity of triangles.