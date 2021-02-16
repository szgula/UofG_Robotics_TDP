from Planner.BasicCommonActions.Action import Action
from GameEngine.src.ball_model import BallBasicModel
from GameEngine.src.robot_model import RobotBasicModel
import numpy as np
import math


class ChaseBallAction(Action):

    def do_algorithm(self):
        return self.chase_ball(self._data[0], self._data[1])

    def chase_ball(self, goalkeeper_model: RobotBasicModel, ball_model: BallBasicModel):
        lookahead = ball_model.get_position()
        pos = goalkeeper_model.get_position_components_wcs()
        angle = goalkeeper_model.get_pointing_angle_wcs()
        curv = self.curvature(lookahead, pos, angle)
        width = float(goalkeeper_model.axis_len)
        wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
        return wheels[1], wheels[0]

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))
