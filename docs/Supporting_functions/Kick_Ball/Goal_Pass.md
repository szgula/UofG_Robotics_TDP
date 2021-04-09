# **Actions**

**[Go back to main page](../../Documentation.md)**

## Check if Score the Goal is feasible
   Checking wehther Score the Goal is feasible is the signal for scoring the goal, this function will use the Game information to help robots judge the distance and find the feasible location to kick the ball.     

### How to check if Score the Goal is feasible in the code?
   
   #### The main function：
        check_if_direct_goal_feasible(player_pos_wcs: Position, ball_pos: Position, opponents_pos_wcs: List[Position], team_id: int, vis: bool = False)
   
   This function uses the game information to judge if the direct goal is feasible or not. We can get any position in game information, not only players' position, but also opponents' position and ball position, goals' position. There we use the simplest and most effectively method to judge the the robot who gets the ball should score the goal or not. 
   
   One is the distance, the other is the dangerous zone. In distance aspect, robot should judge the distance with Goal and itself, make sure is less then the threshold. For the dangerous zone, we write a support function [debug_visualize_pass_danger_zones] to judge whether the robot in dangerous zone, which means whether the opponent can intercept the ball simply
   
   Two support functions:
   
        get_intersection_region(ball_position: np.array, player_position: np.array, kick_slope: float)
        
        debug_visualize_pass_danger_zones(points_vis, pos_opponent)
        
   #### Inputs 
   It should be the game information.
   
   #### Outputs
   
        controller.score_goal(game_info, kick_x, kick_y)
        
   Judge from the functions above, if it is ture, which means the robot is close to opponent's goal and find a feasible location to Score the Goal, thus, the robot will score the Goal directly.

### Shortcomings and future improvements:
   Adding more details about status of the ball is necessary, such as, the specific location of the ball, and how many oppenonts are near the ball. The ball can be in our area and opponent's area, when this happens, what strategic should be apodt. 

        
</p>
