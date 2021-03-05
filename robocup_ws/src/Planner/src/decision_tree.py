from player_controller import PlayerController as pc
from game_interfaces.msg import PlayerCommand

class DecisionTree():
    def __init__(self, pc = None):
        self.threshold = 0.1
        self.controller = pc
        pass

    def plan(self, team, enemy, player_id, mode):
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
            if(closest_player == player_id and self.controller.ball_is_free(game_info)):
                return self.controller.go_to_ball(game_info)
            elif(closest_player == player_id and not self.controller.ball_is_free(game_info)):
                return self.controller.intercept(game_info)
            else:
                return self.controller.go_to_strategic_point(game_info)

