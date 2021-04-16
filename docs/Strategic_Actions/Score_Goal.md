# Strategic action: Score the Goal

##### [Go back to main page](../../Documentation.md)

## Score Goal

Score Goal is one of the most strategic important actions for the robots，since it decides which team can win the game directly. Therefore, effective strategies for scoring the Goal should be considered carefully. 

We can see the figure below showed the processes of how the Score Goal works in the logic. 

   <p align="center">
      <img src="../../Images/Score_Goal_flow.png" /><br><br>
      <b>Figure 1: The flow of scoring the goal</b>
   </p>


As mentioned before, our team adopts the Decision Tree to control all the robots logically. In the **Decision Tree**, when the robot gets the ball, it needs to judge how far from the enemy's goal. 

What is more, it also needs to check if ```scoring the goal is feasible```, which is mentioned in the supporting functions. This check function will return **(x, y) coordinate**, which is the place that robot should do ***receive_and_pass_action*()** in. 

We reuse this function here and set the target position to the enemy's goal, thus, passing the ball to the enemy's goal is scoring the goal.


### How to do Score Goal in code?

The process should be 3 main functions, the figure of the theory is showed below:

   <p align="center">
      <img src="../../Images/Score_Goal.png" /><br><br>
      <b>Figure 2: Score goal in virualiztion</b>
   </p>



There three main functions：

#### 1) Check if Team Has Ball

This function returns a flag if any of the team's robot are within contact distance from the ball. 
If so, it returns the identifies.

In code, this function is defined with staticmethod 

```def has_ball(game_info: list)```

The function uses the game information to judge if the robot has the ball and return True or False.
        
#### 2) Check if Robot Can Score the Goal

This function returns a flag if a robot with a ball can score a goal in direct kick. 
If the goal is possible, it also returns the kick coordinates.
It reuses supporting functions to generate points on the net and checking for possible collisions with robots along the kick trajectory.

In code, this function is defined with staticmethod 

```def can_score(game_info: list, team_id) -> (direct_kick_feasible, kick_x, kick_y)```

This function helps to judge whether the recent location is feasible for robots to score a goal, there more details mentioned in the ***can_score()*** individually.



#### 3) Score the Goal

This function is very similar to _Pass the_Ball_ function, and both share most of the code. 
The only difference between mentioned functions is that _Score the Goal_ refers to stationary point on the net instead of another player.

In code, this function is defined with staticmethod 

```def score_goal(game_info,kick_x,kick_y) -> command ```

Judge from the second function ***can_score()*** above, if it is **True**, which means the robot is close to the opponent's goal and find a feasible location to score the goal,  then the robot will ***do score_goal()*** directly.

Finally, we can see the **No. 3** robot in the **Team 0** (Blue one) scores the goal, which is showed in the Gif below:

<p align="center">
   <img src="../../Images/Score_Goal.gif" /><br><br>
   <b>Figure 3: Gif of scoring goal in virualization</b>
</p>



### Shortcomings and future improvements:

Although the robot can simply judge when and how to score the ball, yet the real physical environment is more complex. 
We should also check if there is an opponent near the robot by the limited version. 

Besides, make the robot able to take actions dynamically according to the real-time situation. 
Finally, using some deep learning methods, such as BP and CNN should be the other solution to make the robots smart to react to the ball.
      

