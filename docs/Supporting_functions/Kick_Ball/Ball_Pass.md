# Actions

##### [Go back to main page](../../Documentation.md)

## Check if passing the ball is feasible

To make players work together the pass_ball function is needed. 
As the situation in the field is dynamic, to make the process of passing accurate and safe, the team needs to check if any opponent can capture the ball on the ball trajectory.

### Implementation

Each robot role has different strategies and priorities for passing the ball. 
For example, the defenders focus on controlling the ball and finding a better place to beat back, while the attackers need to put more focus on the opponents. To that end, we introduced an extra threshold value to assess the risk of passing. 
If opponents are too close, a player needs to pass quickly. If not, a player could keep moving forward to find a better place for passing.

![Check Pass Ball workflow](../../Figures/Actions_check_pass_ball_feasible.png)

__Figure 1:__ Check pass ball feasibility.

The algorithm is implement in the following function:
- attacker check (common): ```def check_for_pass(self, game_info: list, net:list) -> [bool, int]```

- defender check (simplified): ```def defender_pass(self, game_info: list, net:list) -> [bool, int]```

#### Inputs & Outputs

- __Inputs:__ dynamics game information, field information
- __Outputs:__ flag if pass is feasible, the identifier of player to pass

#### Threshold

Threshold aims to parametrise the risk of passing. 
This value mainly depends on the distance between a player and opponents. 
If the risk is acceptable the value can be small (1 or smaller), otherwise keep it high.
In the code, this threshold value is used to determine whether the pass of the ball is feasible.