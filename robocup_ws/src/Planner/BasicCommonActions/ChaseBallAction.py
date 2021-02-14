from BasicCommonActions.Action import Action
from ball_model import BallModel
from robot_model import RobotModel
import numpy as np
import math


class ChaseBallAction(Action):

    def do_algorithm(self):
        return self.chase_ball(self._data[0], self._data[1])

    def chase_ball(self, goalkeeper_model: RobotModel, ball_model: BallModel):
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

    def judge_distance_ball(self, lookahead, pos):
        distance_ball = np.sqrt((lookahead[0]-pos[0]) ** 2 + (lookahead[1] - pos[1])**2)
        print('distance_ball:', distance_ball)
        return distance_ball

    def judge_distance_teammates(self, pos, teammates_pos):
        distance_teammates = []
        for i in range(len(teammates_pos)):
            distance_teammates[i] = np.sqrt((teammates_pos[i][0] - pos[0])**2 + (teammates_pos[i][1]-pos[1])**2)
        return distance_teammates

    def judge_distance_opponents(self, pos, opponents_pos):
        distance_opponents = []
        for i in range(len(opponents_pos)):
            distance_opponents[i] = np.sqrt((opponents_pos[i][0] - pos[0])**2 + (opponents_pos[i][1]-pos[1])**2)
        return distance_opponents

    def judge_distance_OurGoal(self, pos, OurGoal_pos):
        distance_ourGoal = np.sqrt((OurGoal_pos[0] - pos[0]) ** 2 + (OurGoal_pos[1] - pos[1]) ** 2)
        return distance_ourGoal

    def judge_distance_TheirGoal(self, pos, TheirGoal_pos):
        distance_TheirGoal = np.sqrt((TheirGoal_pos[0] - pos[0]) ** 2 + (TheirGoal_pos[1] - pos[1]) ** 2)
        return distance_TheirGoal