

# **Actions**

**[Go back to main page](../../Documentation.md)**

## Avoid Obstacle

Avoiding obstacle function ensure that robots do not collide with each other. 
This is necessary when two robots from opponent teams move along trajectories that cross or get close together.  

The obstacle avoidance needs to consider two main scenarios:
- Situations that involve short and high priority actions (i.e., pass the ball) 
  when a robot should minimise the risk of losing an initiative in a short perspective but after a short period it can land in an unfavourable position.
- Cases when robot executes long actions and does not contain short term behavioural twists (example presented in Figure 1).

<p align="center">
   <img src="../../Images/Avoid_obstacle.png" /><br><br>
</p>
__Figure 1__: Visualization of avoid obstacle function

The information flow for the avoid obstacles function is presented in Figure 1.

<p align="center">
   <img src="../../Images/avoid_obstacle_flow.png" /><br><br>
</p>
__Figure 2__: The information flow to avoiding obstacle function


   

### Avoid obstacles methods

Avoid obstacles logic is split into multiple stages of which execution (if and to what extent) is dependent on a player's specific situation.

#### First stage: Go back

The simplest process to avoid obstacles is to roll back the player. 
This method is especially useful (in situations) close to the field edges but has many limitations and strategic applications.
        
#### Second stage: Rotate away from the obstacle

The second stage tries to avoid obstacles based on the assumption that an obstacle is stationary.
It is realised by rotating away from the forward obstacles. This method works fine in cases the obstacle is stationary or moves away from the collision point.
However, due to undertaken assumption, it fails to execute if an obstacle is moving the same way as our robot is rotating. 
To overcome this, we applied changing rotation if this stage fails to execute over a predefined period of time.
Although it does not cover all scenarios, we found it works sufficiently well in the games.

In figure 3, we can see the **No. 0** robot in the **Team 0**(blue team) does avoid obstacle using mentioned method.

   <p align="center">
      <img src="../../Images/Avoid_obstacle.gif" /><br><br>
   </p>
__Figure 3__: Visualisation of avoiding obstacle function


#### Final stage: Keep distance

This stage ensures to not get back to situation when avoid the obstacle is necessary. 
It "pulls away" robot from the obstacles (like potential field method).
<p align="center">
   <img src="../../Images/avoid_obstacle_theory.png" /><br><br>
</p>
__Figure 4__: Keep the distance visualisation

### Implementation

The above described methods are implemented under following static methods:

```python
# The function is to judge the distance between the robot and obstacles (Teammates, opponents, ball).
def distance_judge(game_info: list, init_distance = None) -> [bool, int, int]:

# When the flag of the above function is true, we can get the team id and player id from distance_judge() function and put them here so that the robot can do avoid_obstacle().
def avoid_obstacle(game_info: list, team_id, obstacle_player_id) -> PlayerCommand:
```
