

# **Actions**

**[Go back to main page](../../Documentation.md)**

## Avoid Obstacle

Avoiding obstacle is one of the necessary action for robots, since the robots will get stuck with each other if they do not do avoiding obstacle when they are closed to each other. 

For continuing the football match, robots need to deal with that problem by itself, therefore, one of feasible method is avoiding obstacle.



### 1. When to do avoiding obstacle?

   Two situations need to be considered:

        According to the distance between each robot and obstacle (teammates / opponents).
        
        According to different situations in the decision tree.

   <p align="center">
      <img src="../../Images/avoid_obstacle_flow.png" /><br><br>
      <b>Figure 1: The flow of avoiding obstacle</b>
   </p>




In the theory, if the distance is less than the threshold, then do avoiding obstacle. However, in some special situation, when the robots need to do something meaningful, for example, pass the ball to teammates and score the goal, it is not necessary to do avoiding obstacle. 

   <p align="center">
      <img src="../../Images/Avoid_obstacle.png" /><br><br>
      <b>Figure 2: Avoid obstacle in virualization</b>
   </p>

   

### 2. How to do avoiding obstacle?

   #### First stage: go back

This method seems to be useful for testing. However, when the robot of the opponents go forward and attack, it can not do anything for preventing the opponents.
        
   #### Second stage: Rotate slight away from the obstacle

It should work in the theory. However, in the code, since the program judges the distance every time and re-planning for the robot every time,  therefore if there is moment the robot tends to be more closed to the obstacle, it will be more closer and closer and can't break away from the obstacle.

Here is the Gif of avoiding obstacle, we can see the **No. 0** robot in the **Team 0**(Blue one) does avoid obstacle.

   <p align="center">
      <img src="../../Images/Avoid_obstacle.gif" /><br><br>
       <b>Figure 3: Gif of avoiding obstacle in virualization</b> 
   </p>



   #### Final stage: Keep distance larger than threshold

This method means when if the distance between robot and the obstacle is less than threshold, the robot needs to get rid of the situation at a time so that the next time it can do meaningful action for the match. 

   <p align="center">
      <img src="../../Images/avoid_obstacle_theory.png" /><br><br>
       <b>Figure 4: Final theory of avoiding obstacle</b>
   </p>


### 3. How it works in the code?

Two main parts in the code:

**One of them is :**

​	*Definition:*

```python
# The function is to judge the distance between the robot and obstacles (Teammates, opponents, ball).

@staticmethod
def distance_judge(game_info: list, init_distance = None) -> [bool, int, int]:
```
​	*Usage：*

```python
# It will return the [bool, int, int], the first one is a flag to judge whether the robot meets the obstacle or not. The second one and the third one are team id and player id of the robot which meets obstacle respectively. 

is_obstacle, which_team, obstacle_id = controller.distance_judge(game_info)
```

**The  other one of them is:** 

​	*Definition：*

```python
# When the flag of the above function is true, we can get the team id and player id from distance_judge() function and put them here so that the robot can do avoid_obstacle().

@staticmethod
def avoid_obstacle(game_info: list, team_id, obstacle_player_id) -> PlayerCommand:
```

​	*Usage：*

```python
# Just fill with information in the funtion, which needs to be metioned here is that the avoid_obstacle() function reuses the go_around_the_point() function, but changes the parameters.

controller.avoid_obstacle(game_info, which_team, obstacle_id)
```



### 4. Shortcomings and future improvements:

It is considered that combine the avoid obstacle and dribble together, if the robot can dribble and avoid obstacle, it will the professional robotic football player and the match will be more interesting.

Furthermore, the vision and other sensors are very necessary for the function, it is difficult for robots to judge whether do avoiding obstacle just according to the distance, the information is not enough to support it to make a wise decision.  
        
</p>