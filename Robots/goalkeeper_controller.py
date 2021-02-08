import numpy as np
import math
from GameEngine.ball_model import BallModel
from GameEngine.robot_model import RobotModel
from Robots.robot_control import Robot, Goal


class GoalkeeperController(Robot):

    def __init__(self):
        self._goalkeeper = None
        self._ball = None
        self._lookhead = 10

    def get_action(self, goal) -> tuple:
        if goal == Goal.ChaseBall:
            lookahead = self._ball.get_position()

            pos = self._goalkeeper.get_position_components_wcs()

            angle = self._goalkeeper.get_point_angle_wcs()

            curv = self.curvature(lookahead, pos, angle)

            width = float(self._goalkeeper.get_axis_len())

            wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]

            return wheels[1], wheels[0]

    def accrue_sensors_data(self, goalkeeper_model: RobotModel, ball_model: BallModel):
        self._goalkeeper = goalkeeper_model
        self._ball = ball_model

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(3.1415 / 2 - angle) * (lookahead[0] - pos[0]) - math.cos(3.1415 / 2 - angle) * (
                    lookahead[1] - pos[1]))
        a = -math.tan(3.1415 / 2 - angle)
        c = math.tan(3.1415 / 2 - angle) * pos[0] - pos[1]

        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(4.4) ** 2))
