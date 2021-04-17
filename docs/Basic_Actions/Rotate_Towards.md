# Rotate Towards
##### [Go back to main page](../../Documentation.md)

This action implements the open-loop controller which commands the robot to change it's heading angle towards a
given orientation. The function which implements it is called ```rotate_towards()```.
## Inputs and Outputs
__Inputs__: Robot position, Target heading

__Output__: Robot control command (single instance)

## Logic
In a differential drive, the vehicle rotates about itself in a circular motion if the angular speeds are equal in magnitude
but opposite in direction. Therefore the the angular speed is given by the following logic using a open loop propotional
controller:

```
vel_l = angle_diff * K_P_PURE_ROTATION
vel_r = - angle_diff * K_P_PURE_ROTATION
```


<!-- ![Kick Ball Vector Diagram](../Figures/.png) -->