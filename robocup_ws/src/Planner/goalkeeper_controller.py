import numpy as np
import math

from BasicCommonActions.BasicCommonActionsHandler import BasicCommonActionsHandler
from BasicCommonActions.ChaseBallAction import ChaseBallAction
from BasicCommonActions.DoNothingAction import DoNothingAction
from ball_model import BallModel
from robot_model import RobotModel
from robot_control import Robot, Goal


class GoalkeeperController(Robot):

    def __init__(self, goalkeeper_model: RobotModel, ball_model: BallModel):
        self._goalkeeper = goalkeeper_model
        self._ball = ball_model
        self._lookhead = 10
        self._bscActHander = BasicCommonActionsHandler(DoNothingAction([goalkeeper_model]))

    def get_action(self, goal) -> tuple:
        if goal == Goal.ChaseBall:
            self._bscActHander.action = ChaseBallAction([self._goalkeeper, self._ball])
        return self._bscActHander.handle()

    def accrue_sensors_data(self):
        pass
