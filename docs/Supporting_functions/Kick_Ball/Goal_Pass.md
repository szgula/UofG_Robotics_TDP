# **Actions**

**[Go back to main page](../../Documentation.md)**

## Check if score the Goal is feasible
   Checking wehther Score the Goal is feasible makes sense if our team gets the ball, which means we can attack directly with the ball. However, if our team do not get the ball.   

   
### How to do Score Goal in code?
   
   #### The main function：
        check_if_direct_goal_feasible(player_pos_wcs: Position, ball_pos: Position, opponents_pos_wcs: List[Position], team_id: int, vis: bool = False)
   This function uses the game information to judge if the robot has the ball, there are more details in [has_ball].
        
   #### Inputs 
   It should be the played po
   
   #### Outputs
        controller.score_goal(game_info, kick_x, kick_y)
   Judge from the functions above, if it is ture, which means the robot is close to opponent's goal and find a feasible location to Score the Goal, thus, the robot will score the Goal directly.

### Shortcomings and future improvements:
   Adding more details about status of the ball is necessary, such as, the specific location of the ball, and how many oppenonts are near the ball,

        
</p>
