import numpy as np
import math
from src.ball_model import BallModel
from src.robot_model import RobotModel
from Planner.robot_control import Robot, Goal


class GoalkeeperController(Robot):

    def __init__(self):
        self._goalkeeper = None
        self._ball = None
        self._lookhead = 10
        self.flag = False

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
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]

        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))

    def judge_distance(self, lookahead, pos):
        distance = np.sqrt((lookahead[0] - pos[0]) ** 2 + (lookahead[1] - pos[1]) ** 2)

        if distance <= 2.5:
            flag = True



