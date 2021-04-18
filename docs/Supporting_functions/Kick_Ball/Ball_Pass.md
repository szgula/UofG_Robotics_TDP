# Actions

##### [Go back to main page](../../Documentation.md)

## Check if pass the ball is feasible

To make players work together the pass_ball function is needed. 
As the situation in the field is dynamic, to make the process of passing accurate and safe the logic is needed to check if any opponent can capture the ball at the ball trajectory.

### Implementation

Different roles have different strategies and different pass priorities. 
For defenders, we can consider the threat of opponents less because the main purpose of defenders is to control the ball and find a better place to beat back.

However, the attackers need to think more about opponents, so an extra threshold value to access the risk of passing was introduced. 
If opponents are too close, a player needs to pass quickly. If not, a player could keep forward to find a better place to pass.

![Check Pass Ball workflow](../../Figures/Actions_check_pass_ball_feasible.png)

__Figure 1:__ Check Pass Ball Feasible.

The algorithm is implement in the following function:
- attacker check (common): ```def check_for_pass(self, game_info: list, net:list) -> [bool, int]```

- defender check (simplified): ```def defender_pass(self, game_info: list, net:list) -> [bool, int]```

#### Inputs & Outputs

- __Inputs:__ dynamics game information, field information
- __Outputs:__ flag if pass is feasible, the identifier of player to pass

#### Threshold

Threshold aims to parametrise the risk of pass. 
This value mainly depends on the distance between player and enemy players. 
If the high risk is acceptable the value can be small (1 or smaller), otherwise keep it high.
In the code, this threshold value is used to determine whether the pass of the ball is feasible.
