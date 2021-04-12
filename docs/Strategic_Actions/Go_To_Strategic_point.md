# **Actions**

**[Go back to main page](../../Documentation.md)**

## Go To Strategic Point

Strategic Point is somewhere the robot can cooperate with its teammates effectively. Since ball has three basic status, in our team, in opponent's team, free. Therefore, the strategic point should be changed according to different situations. 

However, we consider the ball in the opponent's team is more dangerous and urgent.
When the ball is in our team, what we need to do is to make the other attacker which does not have the ball keeps a distance with the robot who gets the ball so that they can pass the ball to each other and score the ball.  

The flow of ***go_to_strategic_point()*** :

   <p align="center">
      <img src="../../Images/Go_To_Strategic_Point_flow.png" /><br><br>
       <b>Figure 1: The flow of going to strategic point</b>
   </p>

   

### How to do Go To Strategic Point in code?

   <p align="center">
      <img src="../../Images/Go_To_Strategic_Point.png" /><br><br>
       <b>Figure 2: Go to strategic point in virualization</b>
   </p>

  

 The process should be 2 main functions

   #### The first function：
```python
# This funtion returns the speed of wheels of the robot
# return (speed of left wheel, speed of right wheel)

def go_to_parametrized(robot_state: Position, target: Position, MIN_PURE_ROTATION_ANGLE, K_P_PURE_ROTATION, MAX_OUTPUT_VALUR, K_P_FORWARD_COMPONENT):
```

This function will adjust the velocity according to the parameters, such as State Position and Target Position.  Which should be mentioned here is the ***boost()*** function sets and optimizes the parameters so that it can make the robots run smoothly.
        
   #### The Second function：
```python
# return Playercommand(lv, rv, 0)

@staticmethod
def go_to_strategic_point(game_info, partner_id):
```

***go_to_strategic_point()*** function adjusts the velocity of two wheels according to the  position we set so that the robot can go to the strategic point quickly and accurately. For example, if one robot holds the ball, the other one will use ***go_to_strategic_point()*** to get a good position to wait for the ball, so they can work with each other effectively.

Finally, we can see the **No. 3** robot in the **Team 0** (Blue one) goes to the strategic point and its teammate **No. 2** robot gets the ball then they also do pass the ball naturally, which is showed in the Gif below:

   <p align="center">
      <img src="../../Images/Go_To_Strategic_Point.gif" /><br><br>
		<b>Figure 3: Gif of going to the strategic point</b>
   </p>



### Shortcomings and future improvements:

Now we just regard a specific point as the strategic point, and test it by using fixed point relative to the other robot. Although it works in most of cases, yet in some case, it does not make sense to go to that point, for example, when the robot is very closed to the edge of the football field, there is no need for the other one to get that place to cooperate with the robot. 

Therefore, the strategic point should be defined with more parameters and more information. It should be more related to team strategic point, not individual strategic point or the strategic point just for two members.
        
</p>