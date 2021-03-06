from player_controller import PlayerController as pc
from game_interfaces.msg import PlayerCommand
import numpy as np

#This class is the main stratgey structure for team 0 (can be any team)
#It follows a strucurized approach to generalize with any player on the field
#Hence not having any controller specific to one player
#This class will choose a strategy to approach the match and assign functions to each and every player based on
#the important parametet => *game_info*

class DecisionTree():
    def __init__(self, pc = None):
        self.threshold = 0.1
        self.controller = pc
        pass

    def plan(self, team: list, enemy: list, player_id: int, mode: str):
        game_info = [team, enemy, player_id, mode]

        if(self.controller.has_ball(game_info)):
            
            if(self.controller.if_able_score(game_info)):
                self.controller.score(game_info)
            else:
                can_pass, pass_candidate = self.controller.check_for_pass(game_info)
                if(can_pass):
                    return self.controller.pass_ball(game_info,pass_candidate)
                else:
                    self.controller.dribble(game_info) #TODO DRIBBLE PLAYER-BALL
        else:
            closest_player = self.controller.closest_to_ball(game_info)
            ball_is_free, enemy_id = self.controller.ball_is_free(game_info)
            # print(ball_is_free, enemy_id)
            take_turn = np.random.randint(0,2) #Add some random activity
            if(closest_player == player_id):
                # print("GO TO BALL! " + str(closest_player))
                return self.controller.go_to_ball(game_info)
            elif(closest_player != player_id and not ball_is_free):
                # print("INTERCEPT! " + str(player_id))
                # if(take_turn == 1):
                return self.controller.intercept(game_info, enemy_id)
                # else:
                #     return self.controller.go_to_strategic_point(game_info)
            else:
                return self.controller.go_to_strategic_point(game_info)

