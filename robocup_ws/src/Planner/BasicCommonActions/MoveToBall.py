from Planner.BasicCommonActions.Action import Action
from GameEngine.src.ball_model import BallModel
from GameEngine.src.robot_model import RobotModel
import numpy as np
import math


def move_to_point(robot_model: RobotModel, ball_model: BallModel):
    lookahead = ball_model.get_position()
    pos = robot_model.get_position_components_wcs()
    angle = robot_model.get_pointing_angle_wcs()
    side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
    speed = 0.5
    wheels = [speed, -speed]
    if side == 0:
        wheels = [0, 0]
    elif side < 0:
        wheels = np.array(side) * wheels
    return wheels[1], wheels[0]


class MoveToPoint(Action):

    def do_algorithm(self):
        return move_to_point(self._data[0], self._data[1])

