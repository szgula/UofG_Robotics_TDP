from Planner.BasicCommonActions.Action import Action
from GameEngine.src.ball_model import BallBasicModel
from GameEngine.src.robot_model import RobotBasicModel
import numpy as np
import math

# this is not actual class - this is method agregator
class ChaseBallAction(Action):

    def do_algorithm(self):
        return self.chase_ball(self._data[0], self._data[1])

    def chase_ball(self, goalkeeper_model,ball_model):
        lookahead = ball_model.get_position()
        print(f"Lookahead: {lookahead}")
        pos = goalkeeper_model._get_position_components_wcs()
        print(f"Position: {pos}")
        angle = goalkeeper_model._get_pointing_angle_wcs()
        print(f"Angle: {angle}")
        curv = self.curvature(lookahead, pos, angle)
        print(f"Curvature: {curv}")
        width = float(goalkeeper_model.axis_len)
        print(f"Width: {width}")
        wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
        print(f"Wheels: {wheels}")
        return wheels[1], wheels[0]

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))
