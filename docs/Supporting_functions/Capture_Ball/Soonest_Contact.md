# Supporting functions: Capture the ball: Ball the soonest contact

**[Go back to main page](../../../Documentation.md)**


### Get ball's soonest contact
The `soonest contact` function estimates the time a specific player needs to get to the freely moving ball (i.e., when the robot can cross the ball trajectory).
It takes into consideration the ball state (position and velocity), and a robot state (position and max speed). 

If the robot cannot get to the ball before it hits the wall, the function can be configured to take into consideration the ball bounces and calculate intersection time after the ball bounced. 


### Assumptions, Inputs and Outputs
__Assumptions__: The ball is moving with uniform linear motion until it bounces from the field side

__Inputs__: Robot state, Ball state, Field size

__Output__: The earliest time when the robot can intersect with the ball trajectory.

### Logic

1. Ball trajectory is defined as a time parametric function (straight line: ```x(t) and y(t)```).
2. Robot possible position is defined as a time parametric function (circle: ```radios = max_speed * t```, ```x(t)``` and ```y(t)```)
3. The mentioned equations are solved to find the tangent of ball trajectory (line) to robot possible position (circle)
4. If the time solution is negative or complex:
   * Find trajectory after the ball bounce
   * Go back to point 3
    

The visualisations of time solutions (before the first bounce) are presented in Figure 1 and Figure 2, both with different initial ball velocities.

![Behavioural Design](../../../Images/time_to_ball_1.png)
__Figure 1:__ Time solutions (colour) for robots in different initial position (x and y field position) for ball moving with constant velocity (vx > 0, vy = 0) (reduced to solutions found before first bounce).
![Creational Design](../../../Images/time_to_ball_2.png)
__Figure 2:__ Time solutions (colour) for robots in different initial position (x and y field position) for ball moving with constant velocity (vx > 0, vy < 0) (reduced to solutions found before first bounce).