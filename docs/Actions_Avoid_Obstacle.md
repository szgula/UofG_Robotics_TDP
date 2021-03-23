# **Actions**

**[Go back to main page](../Documentation.md)**

## Avoid Obstacle

Avoiding obstacle is one of the necessary action for robots, since the robots will get stuck with each other if they do not do avoiding obstacle when they are closed to each other. 
For continuing the football match, robots need to deal with that problem by itself, therefore, one of feasible method is avoiding obstacle.

There are two problems to be considered:
1. When to do avoding obstacle?
   Two situations need to be considered:
        According to the distance between each robot and obstacle (teammates / opponents).
        According to different situations in the decision tree.
   
   In the theory, if the distance is less than the threshold, then do avoiding obstacle. However, in some special situation, when the robots need to do something meaningful, for example, pass the ball to teammates and score the goal, it is not necessary to do avoiding obstacle. 
   
2. How to do avoding obstacle?
   First stage: go back
        This method seems to be useful for testing. However, when the robot of the opponents go foraward and attack, it can not do anything for preventing the opponents.
        
   Second stage: Rotate slight away from the obstacle
        It should work in the theory. However, in the code, since the program judges the distance every time and replanning for the robot every time,  therefore if there is moment the robot tends to be more closed to the obastacle, it will be more closer and closer and can't break away from the obstacle.
   
   Final stage: Keep distance larger than threshold 
        This method means when if the distance between robot and the obstacle is less than threshold, the robot needs to get rid of the situation at a time so that the next time it can do meaningful action for the match. 
</p>
