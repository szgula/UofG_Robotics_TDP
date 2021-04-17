# Result Analysis

##### [Go back to main page](../Documentation.md)

## Test Cases
1. Go to point  

||  Steps   | Expect Result  | Result |
| ----  |   ----  | ----  | ----  | 
|1|  Set a target point and let player from a random position to start moving | The player go straight to the target point |Pass <p align="center"><img src="../Images/test_go_to_position.gif"/><br><br>figure : go to point test</p>

2. Go around point  

||  Steps   | Expect Result  | Result |
| ----  |   ----  | ----  | ----  | 
|1|  Set a target point and let player from a random position to start moving | The player will go around the target point until find the best angle towards opponent gate |Pass <p align="center"><img src="../Images/test_go_around_point.gif"/><br><br>figure : go to point test</p>

3. Kick ball  

||  Steps   | Expect Result  | Result |
| ----  |   ----  | ----  | ----  | 
|1|  Set player to a random position and no enemy in the way. Let player to kick the ball| The player will find a appropriate angle and shoot|Pass <p align="center"><img src="../Images/test_kick_no_enemy.gif"/><br><br>figure : test kick</p>
|2|  Set player to a random position and one enemy in the way. Let player to kick the ball | The player will find a appropriate angle and shoot |Pass. And the shoot angle become larger<p align="center"><img src="../Images/test_kick_one_enemy.gif"/><br><br>figure  : test kick with one enemy</p>
|3|  Set player to a random position and four enemy in the way. Let player to kick the ball | The player will find a appropriate angle and shoot |Pass. And the shoot angle become larger <p align="center"><img src="../Images/test_kick_four_enemy.gif"/><br><br>figure : test kick with four enemy</p>
|4| Set player to a random position and five enemy in the way, which give no space and chance to let the player kick the ball | The player will just dribble the ball or wait other team member |Pass <p align="center"><img src="../Images/test_kick_five_enemy.gif"/><br><br>figure: test kick with five enemy</p>


4. Dribble 

||  Steps   | Expect Result  | Result |
| ----  |   ----  | ----  | ----  | 
|1|  Set the ball to a random position and let player from a random position to start moving | The player will dribble the ball to the opponent gate direction |Pass <p align="center"><img src="../Images/test_dribble.gif" /><br><br>dribble test</p>
|2|  Set player in upper or lower field | The player will dribble the ball along corresponding path  to the gate|Pass <p align="center"><img src="../Images/dribble_from_different_direction.gif" /><br><br>figure :dribble from different direction</p>

