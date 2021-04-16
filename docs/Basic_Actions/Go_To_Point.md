# Go To Point
##### [Go back to main page](../../Documentation.md)

This action implements the open-loop controller which commands the robot to a given location. 
The main function which implements it is called ```go_to_paramaterized()```, 
and few more functions ```go_to_...()``` are used to wrap the main function with different parameters.

## Application - Basic actions: 
__Comment!__ Move to the main file

The Basic Action functions create an Adapter Interface between the robot model (or hardware) and Strategics Controllers.  
It abstracts away the robot underlying physics of differential drive kinematics and allows users to move the robot from one point to another.
Also, separation of those functions away from main team logic ensures easy component integration of new models or physical robots.
In such a case, only this module (Basic Actions) needs to be tuned/created for new players' representation.


### Assumptions, Inputs and Outputs
__Assumptions__: The robot model is known.

__Inputs__: Robot state, Target Position

__Output__: Robot control command (single instance)

## Logic

The working principle behind this function is a sum of two proportional controllers. 
The output of the instantaneous angular speeds is calculated on the basis of the distance between the robot and the target and the angular orientation of the target with respect to the robot. 

In this function, the angular speeds imparted to the wheels is treated as a linear combination of rotational effects and translation effects.

__Comment__: Add diagram

### Rotational Component
In a differential drive, the vehicle rotates about itself in a circular motion if the angular speeds are equal in magnitude
but opposite in direction. Therefore the pure rotational component of the angular speed is given by the following logic:

```
rotation_component_l = angle_diff * K_P_PURE_ROTATION
rotation_component_r = - angle_diff * K_P_PURE_ROTATION
```


### Forward Component
The forward component of the wheel speeds is responsible to move the robot towards the target. It is proportional to the distance between the robot and the target and is the same for both wheels.

### Linear Combination of the Rotational Component and the Forward Component.
The final angular speeds imparted to the wheels is the addition of the corresponding rotational and forward components. It can be
said that the difference in the speeds of the two wheels is produced by the rotational components, which changes the heading
angle. Hence the final equations for the angular speeds for left and right wheels are:

```
vel_l = rotation_component_l + forward_component
vel_r = rotation_component_r + forward_component
```


<!-- ![Go to point diagram](../Figures/.png) -->
