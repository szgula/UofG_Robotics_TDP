# Actions

##### [Go back to main page](../../Documentation.md)

## Check if pass the ball is feasible

To make players work together more closely and smartly, pass ball is one of the most important actions. However the situation in field changes at every time. To make the process of passing acurrately and safely, we designed this check action for passing ball.

### Workflow

Different roles have different strategies. For defenders, we can consider the threat of opponents less because the main purpose of defenders is to control the ball and find a better place to beat back.

However for attackers, we need think more about opponenets, so here we also designed a threshold value to access the risk of passing. If opponents are too close, our team players need to pass quickly. If not, our players could keep forward to find a better place to pass.

![Check Pass Ball workflow](../../Figures/Actions_check_pass_ball_feasible.png)

**Figure 1:** Check Pass Ball Feasible

### How to check the safety of pass in code?

#### The main functions

- attacker check (common)

```
def check_for_pass(self, game_info: list, net:list) -> [bool, int]
```

- defender check (simplied)

```
def defender_pass(self, game_info: list, net:list) -> [bool, int]
```

#### Inputs

```
game_info: list, net:list
```

- game_info: ball position, team players info, opponent team players info, field info.
- net: opponent net info.

#### Outputs

```
can_pass: bool, pass_candidate_id:int
```

- can_pass: whether can pass ball or not after access the current situation.
- pass_candidate_id: the player who pass to.

#### Threshold

Threshold is aimed to access the risk of pass. And this value is mainly depend on the distance between player and enemy players. If the risk is high, we give 1 to threhold. If not, then 2.

```
distance_player_enemy = self.get_closest_opponent(game_info)
if(distance_player_enemy < 5):
    self.strategic_threshold = 2
else:
    self.strategic_threshold = 1
```

Finally, we use this threshold to determine whether pass the ball is feasible

```
if(delta_strategic_point <= self.strategic_threshold):
    return True, pass_candidate
return False, pass_candidate
```
