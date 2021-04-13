# Go Around Point
##### [Go back to main page](../../Documentation.md)
This action is implemented in the function `go_around_the_point`. It generates wheel velocities for robot to go around a point
assuming the robot is already on the circle around the point. The function takes in the radius of curvature of it's path.
The function also takes in the direction of rotatation:
`direction: 1 = clockwise, -1 = counterclockwise`

In differential drives the radius of curvature, the distance between wheels and the ratio of velocities of the left and the
right wheels are related. The relation is given by the following equations:

`Vl/Vr = (R - l/2)/(R + l/2)`

where `R` is the radius of curvature of the path and `l` is the distance between the two wheels.

In this function we take the velocity of the faster wheel is `3`, and the velocity of the slower wheel is calculated
from the ratio mentioned in the equation above.

<!--- ![Go Around Point Diagram](../Figures/.png) -->
