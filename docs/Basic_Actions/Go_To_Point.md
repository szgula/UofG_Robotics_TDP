
# Go To Point
##### [Go back to main page](../../Documentation.md)

This action implements an open-loop controller that commands the robot to a given location. 
The main implements it in function ```go_to_paramaterized()```, 
and other functions in the format of ```go_to_...()``` are used to wrap the main function with different parameters.

## Application - Basic Actions: 
__Comment!__ Move to the main file

The Basic Action functions create Adapter Interface between the robot model (or hardware) and Strategics Controllers.  
It abstracts away the robot underlying physics of differential drive kinematics and allows users to move the robot from one point to another.
Also, separation of those functions away from main team logic ensures easy component integration of new models or physical robots.
In such a case, only this module (Basic Actions) needs to be tuned/created for new players' representations.


### Assumptions, Inputs and Outputs
__Assumptions__: The robot model is known.

__Inputs__: Robot state, Target Position

__Output__: Robot control command (single instance)

## Logic

The logic behind this function is based on a sum of outputs of the two proportional controllers. 
The output of the instantaneous angular speeds is calculated on the basis of the distance between the robot and the target and the angular orientation of the target with respect to the robot. 

In this function, the angular speeds imparted to wheels are treated as a linear combination of rotational and translation effects.

__Comment__: Add diagram

### Rotational Component
In a differential drive, the vehicle rotates around itself if the angular speeds have equal magnitudes but opposite directions. Hence, the pure rotational component of the angular speed is given by the following principle:

```
rotation_component_l = angle_diff * K_P_PURE_ROTATION
rotation_component_r = - angle_diff * K_P_PURE_ROTATION
```


### Forward Component
The forward component of the wheel speeds is responsible for moving the robot towards the target. It is proportional to the distance between the robot and the target and is the same for both wheels.

### Linear Combination of the Rotational Component and the Forward Component.
The final angular speeds attached to the wheels is the addition of the corresponding rotational and forward components. Hence the final equations for the angular speeds for left and right wheels are:

```
vel_l = rotation_component_l + forward_component
vel_r = rotation_component_r + forward_component
```


<!-- ![Go to point diagram](../Figures/.png) -->
