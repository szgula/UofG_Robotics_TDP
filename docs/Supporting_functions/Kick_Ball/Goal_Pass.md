dsf
Is my nake pupa?

# **Actions**

**[Go back to main page](../../../Documentation.md)**

## Check if Score the Goal is feasible

***check_if_direct_goal_feasible()*** is used by methods related to scoring the goal. 
This function returns the flag and a kick position if the goal can be scored to maximise the goal chances.

### Implementation
The information flow of the discussed function is presented in Figure 1.

<p align="center">
  <img src="../../../Images/Goal_Pass_flow.png" /><br><br>
</p>

__Figure 1__: The flow of checking if score the Goal is feasible

The main implementation of the code in under a following static method:
```python
def check_if_direct_goal_feasible(player_pos_wcs: Position, ball_pos: Position, opponents_pos_wcs: List[Position], team_id: int, vis: bool = False)
```

This function uses the game information to judge if the direct goal is feasible or not.
The function takes the following information into consideration:
 - distance from a net, 
 - dangerous zones, representation of the areas where opponents robot can get before the ball cross the net line.

The described method uses multiple supporting functions to calculate the result, including:

1. 
```python
# Estimate the region from where an opponent can capture the ball
# The region is represented by a triangle

@staticmethod
def get_intersection_region(ball_position: np.array, player_position: np.array, kick_slope: float)
```
2.
```python
# This is used to check if an opponent is within the danger zone

@staticmethod
def point_in_triangle(point, t0, t1, t2)
```
