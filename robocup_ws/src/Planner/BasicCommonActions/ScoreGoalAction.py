from BasicCommonActions.Action import Action
from ball_model import BallModel
from ball_model import BallActions
from robot_model import RobotModel
import numpy as np
import math


class ScoreGoalAction(Action):

    def do_algorithm(self):
        return self.score_goal(self._data[0], self._data[1])

    def score_goal(self, goalkeeper_model: RobotModel, ball_model: BallModel):
        lookahead = ball_model.get_position()
        pos = goalkeeper_model.get_position_components_wcs()
        pos_robot = np.array(pos)
        pos_ball = np.array(lookahead)

        goal_post_of_team_2_top = np.array([1000 - 5, 50 + 300])/100
        goal_post_of_team_2_bottom = np.array([1000 - 5, -50 + 300])/100

        goal_post_of_team_2_top = goal_post_of_team_2_top - np.array([10,6]) / 2  
        goal_post_of_team_2_bottom = goal_post_of_team_2_bottom - np.array([10,6]) / 2  

        goal_post_positions = [goal_post_of_team_2_top, goal_post_of_team_2_bottom] #Fixme
        shoot_direction = self.get_shoot_direction(pos_ball, goal_post_positions)
        vantage_point = self.get_vantage_point(pos_ball, shoot_direction)
        if np.linalg.norm(vantage_point - pos_robot) < 0.04:
            return self.shoot()
        else:
            return self.go_to_point(goalkeeper_model, vantage_point)

    def go_to_point(self, goalkeeper_model: RobotModel, point: np.array):
        pos = goalkeeper_model.get_position_components_wcs()
        angle = goalkeeper_model.pointing_angle
        curv = self.curvature(point, pos, angle)
        width = float(goalkeeper_model.axis_len)
        wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
        return wheels[1], wheels[0]

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))

    def get_vantage_point(self, pos_ball, direction_of_shoot):
        position_of_shooter = pos_ball - 0.1 * direction_of_shoot
        return position_of_shooter
    
    def get_shoot_direction(self, ball_pos, goal_post_positions):

        post_rel_top = goal_post_positions[0] - ball_pos
        post_rel_bottom = goal_post_positions[1] - ball_pos

        norm_top = np.linalg.norm(post_rel_top)
        norm_bottom = np.linalg.norm(post_rel_bottom)

        unit_rel_top = post_rel_top/norm_top
        unit_rel_bottom = post_rel_bottom/norm_bottom

        mid_point = (post_rel_top + post_rel_bottom)/2
        norm_mid = np.linalg.norm(mid_point)
        unit_mid_point = mid_point/norm_mid

        if norm_top < norm_bottom:
            direction_of_shoot = unit_mid_point * norm_top + unit_rel_top * norm_mid
            direction_of_shoot = direction_of_shoot/(norm_top + norm_mid)
            direction_of_shoot = direction_of_shoot/np.linalg.norm(direction_of_shoot)
        elif norm_bottom < norm_top:
            direction_of_shoot = unit_mid_point * norm_bottom + unit_rel_bottom * norm_mid
            direction_of_shoot = direction_of_shoot/(norm_bottom + norm_mid)
            direction_of_shoot = direction_of_shoot/np.linalg.norm(direction_of_shoot)
        else:
            direction_of_shoot = unit_rel_top * norm_bottom + unit_rel_bottom * norm_top 
            direction_of_shoot = direction_of_shoot/(norm_bottom + norm_top)
        return direction_of_shoot

    def shoot(self):
        return 0, 0, BallActions.KICK
