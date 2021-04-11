# **Actions**

**[Go back to main page](../../Documentation.md)**

## Avoid Obstacle

Avoiding obstacle is one of the necessary action for robots, since the robots will get stuck with each other if they do not do avoiding obstacle when they are closed to each other. 
For continuing the football match, robots need to deal with that problem by itself, therefore, one of feasible method is avoiding obstacle.

There are two problems to be considered:

### 1. When to do avoding obstacle?

   Two situations need to be considered:
   
        According to the distance between each robot and obstacle (teammates / opponents).
        
        According to different situations in the decision tree.
   
   <p align="center">
      <img src="../../Images/avoid_obstacle_flow.png" />
      <b>Figure 1: The flow of avoiding obstacle<b />
   </p>
   
   In the theory, if the distance is less than the threshold, then do avoiding obstacle. However, in some special situation, when the robots need to do something meaningful, for example, pass the ball to teammates and score the goal, it is not necessary to do avoiding obstacle. 
   
   <p align="center">
      <img src="../../Images/Avoid_obstacle.png" />
      <b>Figure 2: Avoid obstacle in virualization<b />
   </p>
   
   
### 2. How to do avoiding obstacle?
   
   #### First stage: go back
   
   This method seems to be useful for testing. However, when the robot of the opponents go foraward and attack, it can not do anything for preventing the opponents.
        
   #### Second stage: Rotate slight away from the obstacle
   
   It should work in the theory. However, in the code, since the program judges the distance every time and replanning for the robot every time,  therefore if there is moment the robot tends to be more closed to the obastacle, it will be more closer and closer and can't break away from the obstacle.
   
   Here is the Gif of avoiding obstacle, we can see the **No.0** robot in the **Team 0**(Blue one) does avoid obstacle.
   
   <p align="center">
      <img src="../../Images/Avoid_obstacle.gif" />
      <b>Figure 3: Gif of avoiding obstacle in virualization<b />
   </p>
   
   #### Final stage: Keep distance larger than threshold
   
   This method means when if the distance between robot and the obstacle is less than threshold, the robot needs to get rid of the situation at a time so that the next time it can do meaningful action for the match. 
   
   <p align="center">
      <img src="../../Images/avoid_obstacle_theory.png" />
      <b>Figure 4: Final theory of avoiding obstacle<b />
   </p>
   
### Shortcomings and future improvements:
   
   It is considered that comebine the avoid obstacle and dribble together, if the robot can dribble and avoid obstacle, it will the professional robotic football player and the match will be more interesting.
   
   Furthermore, the vision and other sensors are very necessary for this fuction, it is difficult for robots to judge whether do avoiding obstacle just according to the distance, the information is not enough to support it to make a wise decision.  
        
</p>
