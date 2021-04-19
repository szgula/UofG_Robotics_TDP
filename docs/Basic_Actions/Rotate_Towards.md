# Rotate Towards
##### [Go back to main page](../../Documentation.md)

This action implements the open-loop controller that commands the robot to change its heading angle towards a
given orientation. The function is implements in ```rotate_towards()```.

## Inputs and Outputs
__Inputs__: Robot position, Target heading

__Output__: Robot control command (single instance)

## Logic
In a differential drive, the vehicle rotates around itself if the angular speeds have equal magnitudes but opposite directions. Hence, the pure rotational component of the angular speed is given by the following principle:

```
vel_l = angle_diff * K_P_PURE_ROTATION
vel_r = - angle_diff * K_P_PURE_ROTATION
```

