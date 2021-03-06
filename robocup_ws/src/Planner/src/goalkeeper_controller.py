"""import numpy as np
import math

from src.BasicCommonActions.BasicCommonActionsHandler import BasicCommonActionsHandler
from src.BasicCommonActions.ChaseBallAction import ChaseBallAction
from src.BasicCommonActions.DoNothingAction import DoNothingAction
from src.BasicCommonActions.RotateToPointAction import RotateToPointAction"""
from robot_control import Robot, Goal
from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action, go_to_fast
from game_interfaces.msg import Position

"""
class GoalkeeperController(Robot):

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self._lookhead = 10
        self._bscActHander = BasicCommonActionsHandler(DoNothingAction())

    def get_action(self, goal) -> tuple:
        if goal == Goal.ChaseBall:
            self._bscActHander.action = ChaseBallAction([self._goalkeeper, self._ball])
        elif goal == Goal.RotateToPoint:
            self._bscActHander.action = RotateToPointAction([self._goalkeeper, self._ball])
        return self._bscActHander.handle()

    def accrue_sensors_data(self):
        pass
"""


class TempGoalkeeperController(Robot):
    def __init__(self):
        pass

    def get_action(self, my_pos_efcs, ball_pos_efcs):
        l_rpm = 1
        r_rpm = -1
        action = 0
        return PlayerCommand(l_rpm, r_rpm, action)


class Team1GoalkeeperController:  #(Robot)
    def __init__(self, team_id):
        self.read = False
        self.team_id = team_id
        self.vel_scalar = -1
        self.neutral_heading = np.pi / 2
        self.center_of_net = {'x': -5+0.15, 'y': 0.0}
        self.move_radius = 1.2
        self.safe_ball_distance = 4
        self.kick_distance = 0.17

    def get_action(self, my_pos_efcs, ball_pos_efcs, team_positions_wcs=None, opponents_positions_wcs=None):
        ball_from_net_x = ball_pos_efcs.x - self.center_of_net['x']
        ball_from_net_y = ball_pos_efcs.y - self.center_of_net['y']
        d = np.hypot(ball_from_net_x, ball_from_net_y)
        d_player2ball = np.hypot(my_pos_efcs.x-ball_pos_efcs.x, my_pos_efcs.y-ball_pos_efcs.y)

        move_radius = min(self.move_radius, self.safe_ball_distance / self.move_radius)
        if d > move_radius:
            scalar = move_radius / d
        else:
            scalar = 0.1
        x = self.center_of_net['x'] + ball_from_net_x * scalar
        x = max(x, self.center_of_net['x'])
        y = self.center_of_net['y'] + ball_from_net_y * scalar
        l_v, r_v = go_to_fast(my_pos_efcs, Position(x, y, 0))
        action = 0 if d_player2ball > self.kick_distance else 1

        return PlayerCommand(l_v, r_v, action)
