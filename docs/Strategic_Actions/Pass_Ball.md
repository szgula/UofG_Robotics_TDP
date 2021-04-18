
# Strategic action: Pass the ball

##### [Go back to main page](../../Documentation.md)

### Pass the ball

Passing the ball is an essential action that allows robots to collaborate. 
As the name suggests, it is used to intentionally kick the ball towards a teammate.
The possible use cases are, for example, pass the ball to the player closer to the opponent's net, pass the ball to the fellow player if the opponent's robot endangers the player's own initiative.

### Assumptions, Inputs and Outputs
__Assumptions__: The ball is in contact distance from the player, 
and a teammate to pass the ball to is in a clear position (no close-by opponents)

__Inputs__: Robot State, Ball State, Mate Position

__Output__: Robot control signal

### Logic
The pass the ball logic is built on top of ```redirect the ball``` action.

Steps:
* Calculate the player's relative positions (to the ball and mate player)
* While not (robot & mate & ball are inline)
    * Redirect the ball
* Stop the ball and kick
