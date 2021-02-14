import numpy as np
import math
from GameEngine.ball_model import BallModel
from GameEngine.robot_model import RobotModel
from Robots.robot_control import Robot, Goal


class GoalkeeperController(Robot):

    def __init__(self, goalkeeper_model: RobotModel, ball_model: BallModel):
        self._goalkeeper = goalkeeper_model
        self._ball = ball_model
        self._lookhead = 10

    def get_action(self, goal) -> tuple:
        if goal == Goal.ChaseBall:
            lookahead = self._ball.get_position()
            pos = self._goalkeeper.get_position_components_wcs()
            angle = self._goalkeeper.pointing_angle
            curv = self.curvature(lookahead, pos, angle)
            width = float(self._goalkeeper.axis_len)
            wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
            return wheels[1], wheels[0]

    def accrue_sensors_data(self):
        pass

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))
