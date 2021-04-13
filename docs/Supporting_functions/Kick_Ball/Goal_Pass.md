# **Actions**

**[Go back to main page](../../../Documentation.md)**

## Check if Score the Goal is feasible
***check_if_direct_goal_feasible()*** is the support function for ***score_goal()***, this function will use the game information to give a signal to robots so that it can help robots to judge the distance between the robot and goal, besides, it can find the feasible location to kick the ball. 

There is the flow to introduce how it works:

   <p align="center">
      <img src="../../../Images/Goal_Pass_flow.png" /><br><br>
       <b>Figure 1: The flow of checking if score the Goal is feasible</b>
   </p>



### How to check if Score the Goal is feasible in the code?

   #### The main function：
```python
@staticmethod
def check_if_direct_goal_feasible(player_pos_wcs: Position, ball_pos: Position, opponents_pos_wcs: List[Position], team_id: int, vis: bool = False):
```

This function uses the game information to judge if the direct goal is feasible or not. We can get any position in game information, not only players' position, but also opponents' position and ball position, goals' position. Therefore, we can use the simplest and most effective method to know whether the robot should score the goal or not.

One is the distance, the other is the dangerous zone. In distance aspect, robot should judge the distance with Goal and itself, make sure is less then the threshold. 

For the dangerous zone, we write a support function [debug_visualize_pass_danger_zones] to judge whether the robot in dangerous zone, which means whether the opponent can intercept the ball simply



#### Components :

##### The first function:

```python
# Estimate the region from where an opponent can capture the ball
# The region is represented by a triangle

@staticmethod
def get_intersection_region(ball_position: np.array, player_position: np.array, kick_slope: float): 
```

It returns characteristic points of "pass danger zone":

```python
# a = [danger_zone_corner_0 (ball_position),
#        player_to_pass_position (midpoint between corner 1 and 2)
#        danger_zone_corner_1,
#        danger_zone_corner_2]

# a_vis - is visualization array for pyplot (just for debugging)
```



##### The second function:

```python
# Visualise danger zones - for debugging porpoises

@staticmethod
def debug_visualize_pass_danger_zones(points_vis, pos_opponent):
```
This part should be for test, so we can see the real danger zones in the visualization.



##### The  third function: 

```python
# This is used to check if opponent is within danger zone

@staticmethod
def point_in_triangle(point, t0, t1, t2):
```

Combine the functions above, we can not only know the robot should score the goal or not, but also know the coordinate position of the robot where it can score the goal.



### Shortcomings and future improvements:
Adding more details about status of the ball is necessary, such as the specific location of the ball, and how many opponents are near the ball. The ball can be in our area and opponent's area, when different situations happen, we need to consider about what  logic should be put into ***check_if_direct_goal_feasible()***, something similar to the number of layers of the neural network.

</p>
