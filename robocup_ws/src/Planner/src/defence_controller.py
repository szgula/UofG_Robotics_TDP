
from robot_control import Robot, Goal
from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action, go_to_fast
from game_interfaces.msg import Position


class Team1DefenceController:  #(Robot)
    def __init__(self, init_y_pos):
        self.init_y_pos = init_y_pos
        self.kick_distance = 0.17
        self.perfect_neutral_pos_x = -2
        self.net_pos_x = 5

    def get_current_x_target(self, ball_pos_efcs):
        if ball_pos_efcs.x > 0:
            a = -self.perfect_neutral_pos_x / self.net_pos_x
            b = self.perfect_neutral_pos_x
        else:
            a = (self.net_pos_x + self.perfect_neutral_pos_x) / self.net_pos_x
            b = self.perfect_neutral_pos_x
        return a * ball_pos_efcs.x + b

    def get_current_y_target(self, ball_pos_efcs):
        if ball_pos_efcs.y * self.init_y_pos < 0:
            y = 0.15 * np.sign(self.init_y_pos)
        else:
            y = ball_pos_efcs.y
        return y

    def get_goal_keeper_support(self, my_pos_efcs, ball_pos_efcs):
        move_radius = 1.2
        center_of_net = {'x': -5 + 0.15, 'y': 1.0 * np.sign(self.init_y_pos)}
        ball_from_net_x = ball_pos_efcs.x - center_of_net['x']
        ball_from_net_y = ball_pos_efcs.y - center_of_net['y']
        d = np.hypot(ball_from_net_x, ball_from_net_y)
        d_player2ball = np.hypot(my_pos_efcs.x - ball_pos_efcs.x, my_pos_efcs.y - ball_pos_efcs.y)
        if d > move_radius:
            scalar = move_radius / d
        else:
            scalar = 0.1
        x = center_of_net['x'] + ball_from_net_x * scalar
        x = max(x, center_of_net['x'])
        y = center_of_net['y'] + ball_from_net_y * scalar
        l_v, r_v = go_to_fast(my_pos_efcs, Position(x, y, 0))
        action = 0 if d_player2ball > self.kick_distance else 1

        return PlayerCommand(l_v, r_v, action)


    def get_action(self, my_pos_efcs, ball_pos_efcs, team_positions_wcs=None, opponents_positions_wcs=None):
        d_player2ball = np.hypot(my_pos_efcs.x-ball_pos_efcs.x, my_pos_efcs.y-ball_pos_efcs.y)
        if ball_pos_efcs.x < -4:
            command = self.get_goal_keeper_support(my_pos_efcs, ball_pos_efcs)
        else:
            x = self.get_current_x_target(ball_pos_efcs)
            y = self.get_current_y_target(ball_pos_efcs)
            l_v, r_v = go_to_fast(my_pos_efcs, Position(x, y, 0))
            action = 0 if d_player2ball > self.kick_distance else 1
            command = PlayerCommand(l_v, r_v, action)
        return command


