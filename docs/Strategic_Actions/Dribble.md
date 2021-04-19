# **Actions**

**[Go back to main page](../../Documentation.md)**

## Dribble a Ball

Dribbling a ball is an advanced action for robots that allows them to carry a ball towards the opponent's net. 
With the ball closer to the opponent's net, the probability of scoring the goal is higher.
This action is formed by two basic actions: receiving a ball and going to a point. 

### Dribbling Logic

When the player wants to dribble the ball, the first thing that needs to be considered is the speed of the ball. The logic utilises the receive_ball method to synchronise the ball speed with the player's speed.

Next, the player needs to find an appropriate angle towards the opponent net. We use redirect_ball method to rotate the robot and a ball towards the desired point (i.e., the position for goal scoring or avoiding obstacles).

Finally, we select two points located in the corners of the net's area and the player chooses the closest one.

<p align="center">
   <img src="../../Images/dribble_ball_goal_points.png" />
</p>

__Figure 1__: Dribble target points.
   
### Application in the Decision Tree
<p align="center">
  <img src="../../Images/Decision_Tree_1.svg" />
</p>

__Figure 2__: Application of dribble action in the decision tree.

As shown in Figure 2, we place this advanced action as a second priority. The play should shoot if there is any chance to score. In situations when scoring is not feasible (e.g., the player is stuck or is too far to the net) the best action is to dribble the ball towards a more optimal position.

### Schematic Map

<p align="center">
  <img src="../../Images/dribble_ball_schematic_map.jpg" />
</p>

__Figure 3__: Dribble the ball schematic map.
