# Supporting functions: Capture the ball: Ball the soonest collision

##### [Go back to main page](../../../Documentation.md)

### Supporting functions: Move to the main file

Supporting functions are mid-level function used by high-level planners which takes strategic decisions for a specific robot or the whole team.
All logic inside this module can be seen as a library of functions that focus on interpreting the environment rather than provide strategic decisions.

Inside supporting functions, some groups can be distinguished:
 * ___Capture the ball___: set of functions to predict the ball movement without players interactions
 * ___Kick the ball___: set of functions to provide information like where the ball can be passed safely or if the ball can be scored. 


### Ball the soonest contact
To basic information about freely moving ball (without any player in action range) is when the ball will hit the field side and where.
This information is essential for more complex logic like an estimation if any player can get to the ball before it gets to the field side.

### Assumptions, Inputs and Outputs
__Assumptions__: The ball is moving with uniform linear motion 

__Inputs__: Ball state, field size

__Output__: Time of ball hitting the field side

### Logic

If the ball velocity is zero: return infinity

Else: calculate the distance along the trajectory to the end of the field and divide by velocity
