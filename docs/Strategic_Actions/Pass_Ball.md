
# Strategic action: Pass the ball

##### [Go back to main page](../../Documentation.md)

### Pass the ball

Pass the ball is an essential action to enable robots to collaborate. 
As the name suggests, it is used to intentionally kick the ball towards the teammate.
The possible use cases are, for example, pass the ball to the player closer to the opponent's net,
or pass the ball to the fellow player if the opponent's robot endangers the player's own initiative.

### Assumptions, Inputs and Outputs
__Assumptions__: The ball is in contact distance from the player, 
and the teammate to pass is in clear position (no close-by opponents)

__Inputs__: Robot state, Ball state, Mate position

__Output__: Robot control signal

### Logic
The pass the ball logic is built on top of ```redirect the ball``` action.

Steps:
* Calculate the player's relative positions (to the ball and mate player)
* While not (robot & mate & ball are inline)
    * redirect the ball
* Stop the ball and kick