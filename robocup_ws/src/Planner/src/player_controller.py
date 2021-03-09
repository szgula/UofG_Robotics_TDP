from game_interfaces.msg import PlayerCommand
import numpy as np
import matplotlib.pyplot as plt
from BasicCommonActions.go_to_action import simple_go_to_action
from game_interfaces.msg import Position
from brachistochrone import cycloid
from BasicCommonActions.go_to_action import go_to_fast, \
    receive_and_pass_action,\
    rotate_towards,\
    calculate_angle_difference, \
    clip_angle, \
    boost
from BasicCommonActions.plan_supporting_functions import TeamMasterSupporting
import math
from copy import deepcopy

#This class represents a generic controller for any type of player (excluding the goal keeper)
#This way this code would be updated as a low level functionality everytime a new player is added
#Having us just worry about adding extra functionalities instead of coding all over again

class PlayerController:  #(Robot)
    #All in all, this class contains all actions that players can perform within the field.
    def __init__(self, player_id: int):
        self.read = True
        self.player = player_id
        self.points_to_visit = []
        self.current_goal = 0
        self.goal_threshold = 0.2
        self.strategic_threshold = 1
        self.intercept_threshold = 0.2
        self.headed_threshold = 0.3
        self.cover_threshold = 0.6

    def has_ball(self, game_info: list) -> bool:
        team = game_info[0]
        player_id = game_info[2]
        position = team.players_positions_efcs[player_id]
        ball_pos = team.ball_pos_efcs
        d = np.hypot(ball_pos.x - position.x, ball_pos.y - position.y)
        if (d <= self.goal_threshold):
            return True
        return False

    def can_score(self, game_info: list, team_id):
        team = game_info[0]
        opponent = game_info[1]
        direct_kick_feasible, kick_x, kick_y = TeamMasterSupporting.check_if_direct_goal_feasible(None,
                                                                                                 team.ball_pos_efcs,
                                                                                                 opponent.players_positions_wcs,
                                                                                                 team_id)
        return direct_kick_feasible, kick_x, kick_y

    def score_goal(self,game_info,kick_x,kick_y):
        team = game_info[0]
        main_player = team.players_positions_efcs[game_info[2]]
        temp_target_pos = Position(kick_x, kick_y, 0)
        raw_command = receive_and_pass_action(main_player, temp_target_pos, team.ball_pos_efcs, team.ball_vel_efcs)
        command = PlayerCommand(*raw_command)
        return command

    def fastest_to_ball(self, game_info):
        team = game_info[0]
        enemy = game_info[1]
        out_tms = TeamMasterSupporting.get_soonest_contact(team, enemy)
        team_can_get_to_ball, players_capture_time, opponent_can_get_to_ball, opponents_capture_time = out_tms
        player_id_min_time = int(np.argmin(players_capture_time))
        opponent_id_min_time = int(np.argmin(opponents_capture_time))
        capture_pos = TeamMasterSupporting.get_ball_pos_at_time(players_capture_time[player_id_min_time],
                                                                team.ball_pos_efcs,
                                                                team.ball_vel_efcs)
        return capture_pos, player_id_min_time

    def check_for_pass(self, game_info: list, net:list) -> [bool, int]:
        team = game_info[0]
        player_id = game_info[2]
        positions = team.players_positions_efcs
        main_player = positions[player_id]
        ball_pos = team.ball_pos_efcs
        pass_candidate = self.get_team_pass_candidate(positions, player_id, net)
        candidate_pos = positions[pass_candidate]
        candidate_pos_np = np.array([candidate_pos.x, candidate_pos.y])
        strategic_point = deepcopy(ball_pos)
        strategic_point.x = float(strategic_point.x) + 1
        if (ball_pos.y < 0):
            strategic_point.y = float(strategic_point.y) + 0.5
        else:
            strategic_point.y = float(strategic_point.y) - 0.5
        strategic_point_np = np.array([strategic_point.x, strategic_point.y])
        delta_strategic_point = np.linalg.norm(strategic_point_np - candidate_pos_np)
        if(delta_strategic_point <= self.strategic_threshold):
            return True, pass_candidate
        return False, pass_candidate

    def pass_ball(self, game_info: list, candidate: int) -> PlayerCommand:
        team = game_info[0]
        ball_vel = team.ball_vel_efcs
        main_player = team.players_positions_efcs[game_info[2]]
        candidate_pos = team.players_positions_efcs[candidate]
        position = np.array([candidate_pos.x,candidate_pos.y])
        ball = team.ball_pos_efcs
        lv, rv, action = receive_and_pass_action(main_player,candidate_pos,ball, ball_vel)
        return PlayerCommand(lv,rv,action)


    def closest_to_ball(self,game_info: list) -> int:
        team = game_info[0]
        player_id = game_info[2]
        positions = team.players_positions_efcs
        position = [[position.x,position.y] for position in positions]
        position = np.array(position).astype(float)
        ball_pos = np.array([team.ball_pos_efcs.x,team.ball_pos_efcs.y]).astype(float)
        idx = np.argmin(np.linalg.norm(ball_pos-position, axis=1))
        return idx

#This function allows a striker to follow his teammate who is closest or has the ball in a parallel manner
    def go_to_strategic_point(self,game_info, partner_id):
        team = game_info[0]
        player_id = game_info[2]
        ball_pos = team.ball_pos_efcs
        team_positions = team.players_positions_efcs
        main_player = team_positions[player_id]
        partner_player = team_positions[partner_id]
        strategic_point = deepcopy(ball_pos)
        strategic_point.x = float(strategic_point.x) + 1
        if(ball_pos.y < 0):
            strategic_point.y = float(strategic_point.y) + 1
        else:
            strategic_point.y = float(strategic_point.y) - 1
        lv, rv = boost(main_player, strategic_point)
        return PlayerCommand(lv,rv,0) #TODO

    # def check_pass_candidate(self,game_info, :

    def intercept(self,game_info: list, enemy_id: int, net: list) -> PlayerCommand:
        team = game_info[0]
        opponents = game_info[1]
        enemies_positions = opponents.players_positions_wcs
        player_id = game_info[2]
        main_player = team.players_positions_efcs[player_id]
        main_enemy = enemies_positions[enemy_id]
        main_player_np = np.array([main_player.x, main_player.y])
        enemy_candidates = self.get_opponent_pass_candidates(enemies_positions,enemy_id, net)
        candidate_coord_1 = enemies_positions[enemy_candidates[0]]
        candidate_coord_1_np = np.array([candidate_coord_1.x, candidate_coord_1.y])
        candidate_coord_2 = enemies_positions[enemy_candidates[1]]
        candidate_coord_2_np = np.array([candidate_coord_2.x, candidate_coord_2.y])
        ball_pos = team.ball_pos_efcs
        distance_to_ball_x = ball_pos.x - main_player.x
        distance_to_ball_y = ball_pos.y - main_player.y
        heading_angle = np.arctan2(distance_to_ball_y, distance_to_ball_x)
        min_1 = np.min(np.linalg.norm(main_player_np - candidate_coord_1_np))
        min_2 = np.min(np.linalg.norm(main_player_np - candidate_coord_2_np))
        final_min_candidates = min(min_1, min_2)
        if(final_min_candidates == min_1):
            candidate_coord = candidate_coord_1
        else:
            candidate_coord = candidate_coord_2
        kick_slope = (candidate_coord.y - main_enemy.y)/(candidate_coord.x - main_enemy.x)
        danger_clause,_ = TeamMasterSupporting.get_intersection_region(np.array([ball_pos.x,
                                                                               ball_pos.y]),
                                                                     np.array([candidate_coord.x,
                                                                               candidate_coord.y]),
                                                                     kick_slope)
        danger_corner = danger_clause[1]
        lv, rv = go_to_fast(main_player,Position(danger_corner[0],danger_corner[1],0))
        return PlayerCommand(lv,rv,0)

    def ball_in_zone(self, game_info, field_size):
        team = game_info[0]
        ball_pos = team.ball_pos_efcs
        if(ball_pos.x < field_size[0]/2):
            return True
        return False

    def cover(self, game_info: list, net: list):
        team = game_info[0]
        opponents = game_info[1]
        enemies_positions = opponents.players_positions_wcs
        player_id = game_info[2]
        main_player = team.players_positions_efcs[player_id]
        main_player_np = np.array([main_player.x, main_player.y])
        enemy_candidates = self.get_dangerous_opponents(enemies_positions, net)
        candidate_coord_1 = enemies_positions[enemy_candidates[0]]
        candidate_coord_1_np = np.array([candidate_coord_1.x, candidate_coord_1.y])
        candidate_coord_2 = enemies_positions[enemy_candidates[1]]
        candidate_coord_2_np = np.array([candidate_coord_2.x, candidate_coord_2.y])
        ball_pos = team.ball_pos_efcs
        min_1 = np.min(np.linalg.norm(main_player_np - candidate_coord_1_np))
        min_2 = np.min(np.linalg.norm(main_player_np - candidate_coord_2_np))
        final_min_candidates = min(min_1, min_2)
        if (final_min_candidates == min_1):
            candidate_coord = candidate_coord_1
        else:
            candidate_coord = candidate_coord_2
        candidate_coord.x = float(candidate_coord.x) - 0.2
        candidate_coord.y = float(candidate_coord.y) - 0.2
        delta_position = np.hypot(candidate_coord.x - main_player.x, candidate_coord.y - main_player.y)
        if(delta_position <= self.cover_threshold):
            delta_position = np.array([ball_pos.x, ball_pos.y]) - np.array([main_player.x, main_player.y])
            calculated_heading = np.arctan2(delta_position[1],delta_position[0])
            lv, rv, action, _ = rotate_towards(main_player, calculated_heading)
        else:
            lv, rv = go_to_fast(main_player, candidate_coord)
        return PlayerCommand(lv, rv, 0)

    def get_team_pass_candidate(self, team: list, player_id: int, net: list) -> int:
        position = [np.array([position.x, position.y]) for position in team]
        position[player_id] = np.array([np.inf, np.inf])
        team_pass_candidate = np.argmin(np.linalg.norm(position - np.array(net), axis=1))
        return team_pass_candidate

    def get_opponent_pass_candidates(self, enemies: list, enemy_id: int, net: list) -> int:
        position = [np.array([position.x, position.y]) for position in enemies]
        position[enemy_id] = np.array([np.inf, np.inf])
        op1, op2, *_ = np.argpartition((np.linalg.norm(position - np.array(net), axis=1)), 1)
        return [op1, op2]

    def get_dangerous_opponents(self, enemies, net):
        position = [np.array([position.x, position.y]) for position in enemies]
        op1, op2, *_ = np.argpartition((np.linalg.norm(position - np.array(net), axis=1)), 1)
        return [op1, op2]

    def ball_is_free(self,game_info: list) -> [bool, int, int]:
        team = game_info[0]
        opponents = game_info[1]
        ball_pos = np.array([team.ball_pos_efcs.x, team.ball_pos_efcs.y]).astype(float)
        #IF BALL WITH ENEMY
        positions_opponents = opponents.players_positions_wcs
        position_opponents = [[position.x, position.y] for position in positions_opponents]
        position_opponents = np.array(position_opponents).astype(float)
        min_pos_opponents = np.min(np.linalg.norm(position_opponents - ball_pos, axis=1))
        min_arg_opponents = np.argmin(np.linalg.norm(position_opponents - ball_pos, axis=1))
        #IF BALL WITH TEAM
        positions_team = team.players_positions_wcs
        position_team = [[position.x, position.y] for position in positions_team]
        position_team = np.array(position_team).astype(float)
        min_pos_team = np.min(np.linalg.norm(position_team - ball_pos, axis=1))
        min_arg_team = np.argmin(np.linalg.norm(position_team - ball_pos, axis=1))
        final_min = min(min_pos_team, min_pos_opponents)

        if(final_min == min_pos_team):
            if(min_pos_team <= self.intercept_threshold):
                return False, 1, min_arg_team
        else:
            if(min_pos_opponents <= self.intercept_threshold):
                return False, 0, min_arg_opponents
        return True, -1, -1


    def receive_ball(self, robot_state: Position, target: Position, ball_position: Position, ball_vel: Position, my_robot_id, player_to_pass):
        d = np.hypot(robot_state.x - ball_position.x, robot_state.y - ball_position.y)
        action = 0
        if d < 0.17:
            direct_kick_feasible, kick_x, kick_y = TeamMasterSupporting.check_if_direct_goal_feasible(None,
                                                                                                      self.team_position.ball_pos_efcs,
                                                                                                      self.opponents_position.players_positions_wcs,
                                                                                                      self.team_id)
            if direct_kick_feasible and my_robot_id in [self.striker_right_idx, self.striker_left_idx]:
                temp_target_pos = Position(kick_x, kick_y, 0)
                raw_command = receive_and_pass_action(robot_state, temp_target_pos, ball_position, ball_vel)
                command = PlayerCommand(*raw_command)
            else:
                temp_target_pos = self.team_position.players_positions_efcs[player_to_pass]
                raw_command = receive_and_pass_action(robot_state, temp_target_pos, ball_position, ball_vel)
                command = PlayerCommand(*raw_command)
        else:
            command = PlayerCommand(*go_to_fast(robot_state, target), action)
        return command

    # def receive_ball(self, game_info):
    #
    #     team = game_info[0]
    #     player_id = game_info[2]
    #
    #     pass_target = team.players_positions_efcs[player_id]
    #     ball_position = team.ball_pos_efcs
    #
    #     delta_position = np.array([ball_position.x, ball_position.y]) - np.array([pass_target.x, pass_target.y])
    #     calculated_heading = np.arctan2(delta_position[1],delta_position[0])
    #     if(calculated_heading is None):
    #         if(delta_position[1] >= 0):
    #             new_heading = np.pi/2
    #         else:
    #             new_heading = -np.pi/2
    #     else:
    #         new_heading = calculated_heading
    #
    #     vel_l, vel_r, action, _ = rotate_towards(pass_target, new_heading)
    #
    #     return PlayerCommand(vel_l, vel_r, 2)



    def go_to_ball(self,game_info: list) -> PlayerCommand:
        team = game_info[0]
        player_id = game_info[2]
        pos = team.players_positions_efcs[player_id]
        ball_pos = team.ball_pos_efcs
        lv, rv = go_to_fast(pos, ball_pos)
        return PlayerCommand(lv, rv, 0)

    def go_to_ball_special(self,game_info: list) -> PlayerCommand:
        team = game_info[0]
        player_id = game_info[2]
        pos = team.players_positions_efcs[player_id]
        ball_pos = team.ball_pos_efcs
        time = TeamMasterSupporting.get_capture_time(ball_pos,
                                                     pos,
                                                     team.ball_vel_efcs)
        go_to_pos = TeamMasterSupporting.get_ball_pos_at_time(time, ball_pos, team.ball_vel_efcs)
        lv, rv = go_to_fast(pos, go_to_pos)
        return PlayerCommand(lv, rv, 0)

    def ball_headed(self, game_info):
        team = game_info[0]
        receiver = team.players_positions_efcs[game_info[2]]
        ball_pos = team.ball_pos_efcs
        if(abs(team.ball_vel_efcs.x) < 0.0001 and abs(team.ball_vel_efcs.y) < 0.0001):
            time = TeamMasterSupporting.get_time_for_stationary_ball(ball_pos, receiver)
        else:
            time = TeamMasterSupporting.get_time_for_moving_ball(ball_pos, receiver, team.ball_vel_efcs, 0)
        headed_pos = TeamMasterSupporting.get_ball_pos_at_time(time, ball_pos, team.ball_vel_efcs)
        delta_distance = np.hypot(ball_pos.x - receiver.x, ball_pos.y - receiver.y)
        print(delta_distance)
        if(delta_distance<=self.headed_threshold):
            if(game_info[2] == 3):
                print("RECEIVE")
            return True
        if(game_info[2] == 3):
            print("GO TO BALL")
        return False

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead.x - pos.x) - math.cos(angle) * (lookahead.y - pos.y))
        a = -math.tan(angle)
        c = math.tan(angle) * pos.x - pos.y
        x = abs(a * lookahead.x + lookahead.y + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))

    def get_coordinates(self,my_pos_efcs:Position, ball_pos_efcs: Position):
        x_trajectory, y_trajectory, T = cycloid(my_pos_efcs.x + 5,
                                                my_pos_efcs.y + 3,
                                                ball_pos_efcs.x + 5,
                                                ball_pos_efcs.y + 3)
        # plt.plot(x_trajectory,y_trajectory)
        # plt.show()
        for x, y in zip(x_trajectory, y_trajectory):
            self.points_to_visit.append(Position(x, y, 0))

    def get_action(self, my_pos_efcs:Position):
        goal_pos = self.points_to_visit[self.current_goal]
        if np.hypot(goal_pos.x - my_pos_efcs.x, goal_pos.y - my_pos_efcs.y) < self.goal_threshold:
            self.current_goal += 1
            self.current_goal %= 6
        l_rpm, r_rpm, = simple_go_to_action(my_pos_efcs, goal_pos)
        print(f"({l_rpm}, {r_rpm})")
        return PlayerCommand(l_rpm, r_rpm, 0)