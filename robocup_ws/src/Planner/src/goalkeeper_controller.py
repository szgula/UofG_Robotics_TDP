"""import numpy as np
import math

from src.BasicCommonActions.BasicCommonActionsHandler import BasicCommonActionsHandler
from src.BasicCommonActions.ChaseBallAction import ChaseBallAction
from src.BasicCommonActions.DoNothingAction import DoNothingAction
from src.BasicCommonActions.RotateToPointAction import RotateToPointAction"""
from src.Planner.src.robot_control import Robot, Goal
from game_interfaces.msg import PlayerCommand

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
        l_rpm = -1
        r_rpm = 1
        action = 0
        return PlayerCommand(l_rpm, r_rpm, action)
