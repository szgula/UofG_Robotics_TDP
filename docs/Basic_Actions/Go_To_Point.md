# Go To Point
##### [Go back to main page](../../Documentation.md)

This action implements movement of the robot to a specific location. The function which implements it is called
`go_to_paramaterized`. This function takes in the position and heading angle of the robot and the target position.
The function returns the angular speeds of the left and the right wheels for each step.

The working principle behind this function is called a simple proportional controller. The output of the instantaneous
angular speeds is calculated on the basis of the distance between the robot and the target and the angular orientation
of the target with respect to the robot. 

In this function the angular speeds imparted to the wheels is treated as a linear combination 
of rotational effects and translation effects.

## Rotational Component
In a differential drive, the vehicle rotates around a point in circular motion if the angular speeds are equal in magnitude
but opposite in direction. Therefore the pure rotational component of the angular speed is given by the following logic:
`rotation_component_l = angle_diff * K_P_PURE_ROTATION`
`rotation_component_r = - angle_diff * K_P_PURE_ROTATION`
Here the magnitudes of the rotational components are propotional to the angle difference between the robot and the target. Notice
the minus sign in the second equation above, that gives the speeds an equal magnitude and opposite direction to each other.

## Forward Component
The forward component of the wheel speeds is responsible to move the robot in the direction of it's own heading angle.
This is propotional to the distance between the robot and the target. However, in the equation below the distance is 
clipped at value of `1`.
`forward_component = min(d, 1) * K_P_FORWARD_COMPONENT`
The forward component is the same for both the left and the right wheels.

## Linear Combination of the Rotational Component and the Forward Component.
The final angular speeds imparted to the wheels is addition of the corresponding rotational and forward components. It can be
said that the difference in the speeds of the two wheels is produced by the rotational components, which changes the heading
angle. Hence the final equations for the angular speeds for left and right wheels are:
`vel_l = rotation_component_l + forward_component`
`vel_r = rotation_component_r + forward_component`


<!-- ![Go to point diagram](../Figures/.png) -->
