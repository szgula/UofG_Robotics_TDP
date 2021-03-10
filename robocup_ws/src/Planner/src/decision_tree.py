from player_controller import PlayerController as pc
from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.plan_supporting_functions import TeamMasterSupporting
from BasicCommonActions.go_to_action import go_to_fast

#This class is the main stratgey structure for team 0 (can be any team)
#It follows a strucurized approach to generalize with any player on the field
#Hence not having any controller specific to one player
#This class will choose a strategy to approach the match and assign functions to each and every player based on
#the important parametet => *game_info*

class DecisionTree():
    def __init__(self,team_id):
        self.threshold = 0.1
        self.field_size = [10,6]
        self.team_id = team_id
        self.net_pos_x_wcs = 5 if self.team_id == 1 else -5
        self.net_pos_y = TeamMasterSupporting.net_length / 2 - 0.1 if self.team_id == 1 \
            else -TeamMasterSupporting.net_length / 2 + 0.1
        self.net_coords = [self.net_pos_x_wcs, self.net_pos_y]
        self.enemy_net = [5, TeamMasterSupporting.net_length / 2 - 0.1]

    def plan(self, team: list, enemy: list, controllers: list, modes: str, actions):
        player_id = 0
        for controller in controllers:
            mode = modes[player_id]
            game_info = [team, enemy, player_id, mode]
            is_obstacle, which_team, obstacle_id = controller.distance_judge(game_info)
            if (controller.has_ball(game_info)):
                can_score, kick_x, kick_y = controller.can_score(game_info, self.team_id)
                if can_score:
                    actions.append([controller.score_goal(game_info, kick_x, kick_y)])
                else:
                    if(mode == "DEFEND"):
                        can_dribble = controller.check_for_dribble(game_info)
                        if can_dribble:
                            actions.append([controller.dribble_ball(game_info)])
                        else:
                            can_pass, pass_candidate = controller.defender_pass(game_info, self.enemy_net)
                            if (can_pass):
                                actions.append([controller.pass_ball(game_info, pass_candidate)])
                            else:
                                actions.append([PlayerCommand(0, 0, 2)])
                    elif(mode == "ATTACK"):
                        can_dribble = controller.check_for_dribble(game_info)
                        if can_dribble:
                            actions.append([controller.dribble_ball(game_info)])
                        else:
                            can_pass, pass_candidate = controller.check_for_pass(game_info, self.enemy_net)
                            if(can_pass):
                                actions.append([controller.pass_ball(game_info,pass_candidate)])
                            else:
                                actions.append([PlayerCommand(0,0,2)])
            else:
                capture_pos,closest_player = controller.fastest_to_ball(game_info)
                #if team_or_enemy = 1 => ball with teammate else ball with opponent
                ball_is_free, team_or_enemy , player_with_ball = controller.ball_is_free(game_info)
                take_turn = np.random.randint(0,2) #Add some random activity
                #STRIKERS
                if(closest_player == player_id and mode == "ATTACK"):
                    if (is_obstacle):
                        actions.append([controller.avoid_obstacle(game_info, which_team, obstacle_id)])
                    else:
                        actions.append([controller.go_to_ball_special(game_info)])
                elif(closest_player != player_id and mode == "ATTACK"):
                    if (is_obstacle):
                        actions.append([controller.avoid_obstacle(game_info, which_team, obstacle_id)])
                    else:
                        actions.append([controller.go_to_strategic_point(game_info, closest_player)])
                #DEFENDERS
                ball_in_zone = controller.ball_in_zone(game_info, self.field_size)
                if(closest_player == player_id  and mode == "DEFEND" and ball_in_zone):
                    actions.append([controller.go_to_ball_special(game_info)])
                elif(mode == "DEFEND"):
                    if(is_obstacle):
                        actions.append([controller.avoid_obstacle(game_info, which_team, obstacle_id)])
                    else:
                        actions.append([controller.cover(game_info, self.net_coords)]) #COVER OVER DEFENCE
            player_id+=1
        return actions

