from game_interfaces.msg import PlayerCommand
import numpy as np
import matplotlib.pyplot as plt
from BasicCommonActions.go_to_action import simple_go_to_action
from game_interfaces.msg import Position
from brachistochrone import cycloid
from BasicCommonActions.go_to_action import go_to_fast
import math


class PlayerController:  #(Robot)
    def __init__(self, player_id):
        self.read = True
        self.player = player_id
        self.points_to_visit = []
        self.current_goal = 0
        self.goal_threshold = 0.1

    def has_ball(self, game_info):
        team = game_info[0]
        player_id = game_info[2]
        position = team.players_positions_efcs[player_id]
        ball_pos = team.ball_pos_efcs
        d = np.hypot(ball_pos.x - position.x, ball_pos.y - position.y)
        if (d <= self.goal_threshold):
            return True
        return False

    def if_able_score(self, game_info):
        pass

    def check_for_pass(self, game_info):
        team = game_info[0]
        player_id = game_info[2]
        positions = team.players_positions_efcs
        position = [[position.x, position.y] for position in positions]
        main_player = position[player_id]
        position[player_id] = [np.inf, np.inf]
        position = np.array(position).astype(float)
        player_pos = np.array([main_player[0], main_player[1]]).astype(float)
        idx = np.argmin(np.linalg.norm(position - player_pos, axis=1))
        return True, idx

    def pass_ball(self, game_info, candidate):
        team = game_info[0]
        main_player = team.players_positions_efcs[game_info[2]]
        candidate_pos = team.players_positions_efcs[candidate]
        position = np.array([candidate_pos.x,candidate_pos.y])
        ball = team.ball_pos_efcs
        ball_pos = np.array([ball.x, ball.y])
        relative_vector = position - ball_pos
        unit_vector = relative_vector/np.linalg.norm(relative_vector)
        vantage_point = ball_pos - 0.1*unit_vector
        kick = 1
        if(np.linalg.norm(np.array([main_player.x, main_player.y]).astype(float) - vantage_point) <= 0.1):
            print("VANTAGE: " + str(np.linalg.norm(np.array([main_player.x, main_player.y]).astype(float) - vantage_point)))
            return PlayerCommand(0,0,kick)
        lv, rv = go_to_fast(main_player,Position(vantage_point[0],vantage_point[1],0))
        return PlayerCommand(lv,rv,0)


    def closest_to_ball(self,game_info):
        team = game_info[0]
        player_id = game_info[2]
        positions = team.players_positions_efcs
        position = [[position.x,position.y] for position in positions]
        position = np.array(position).astype(float)
        ball_pos = np.array([team.ball_pos_efcs.x,team.ball_pos_efcs.y]).astype(float)
        idx = np.argmin(np.linalg.norm(position-ball_pos, axis=1))
        print(idx)
        return idx

    def go_to_strategic_point(self,game_info):
        return PlayerCommand(0,0,0) #TODO

    def intercept(self,game_info):
        return self.go_to_ball(game_info)

    def ball_is_free(self,game_info):
        team = game_info[0]
        opponents = game_info[1]
        positions = opponents.players_positions_wcs
        position = [[position.x, position.y] for position in positions]
        position = np.array(position).astype(float)
        ball_pos = np.array([team.ball_pos_efcs.x, team.ball_pos_efcs.y]).astype(float)
        min_pos = np.min(np.linalg.norm(position - ball_pos, axis=1))
        if(min_pos > self.goal_threshold):
            return True
        return False


    def go_to_ball(self,game_info):
        team = game_info[0]
        player_id = game_info[2]
        pos = team.players_positions_efcs[player_id]
        ball_pos = team.ball_pos_efcs
        lookahead = ball_pos
        angle = pos.theta
        curv = self.curvature(lookahead, pos, angle)
        width = float(0.2)
        wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
        return PlayerCommand(wheels[1],wheels[0],0)

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