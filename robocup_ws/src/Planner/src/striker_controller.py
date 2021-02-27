from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action
from game_interfaces.msg import Position
from BasicCommonActions.go_to_action import go_to_fast
import math


class Team1StrikerController:  #(Robot)
    def __init__(self):
        self.read = True
        self.points_to_visit = [Position(-1,-1, 0),
                                Position(1, 1, 0),
                                Position(2, 0, 0),
                                Position(0, -2, 0),
                                Position(-5, 2, 0),
                                Position(5, -3, 0)]
        self.current_goal = 0
        self.goal_threshold = 0.1

    def get_action(self, my_pos_efcs:Position, ball_pos_efcs: Position):
        goal_pos = self.points_to_visit[self.current_goal]
        print(goal_pos)

        if np.hypot(goal_pos.x - my_pos_efcs.x, goal_pos.y - my_pos_efcs.y) < self.goal_threshold:
            self.current_goal += 1
            self.current_goal %= 6
        l_rpm, r_rpm, = simple_go_to_action(my_pos_efcs, goal_pos)
        print(f"({l_rpm}, {r_rpm})")
        return PlayerCommand(l_rpm, r_rpm, 0)

    def go_to_point(self, my_pos_efcs:Position, ball_pos_efcs: Position):
        start_x_pos = my_pos_efcs.x
        start_y_pos = my_pos_efcs.y
        heading_angle = my_pos_efcs.theta
        target_x_pos = ball_pos_efcs.x
        target_y_pos = ball_pos_efcs.y
        axis_len = 0.1
        distance = 0.1
        ball_radius = 0.05
        target = [target_x_pos, target_y_pos]
        start = [start_x_pos, start_y_pos]
        wheels = [0, 0]
        start_to_target_distance = ((target[0] - start[0]) ** 2 + (target[0] - start[0]) ** 2) ** 0.5
        if start_to_target_distance > (axis_len + ball_radius + distance):
            curv = self.curvature(start, target, heading_angle)
            wheels = [(2 + curv * axis_len) / 2, (2 - curv * axis_len) / 2]

        print(f"({wheels[1]}, {wheels[0]})")
        return PlayerCommand(wheels[1], wheels[0], 0)

    def curvature(self, start, target, angle):
        side = np.sign(math.sin(angle) * (target[0] - start[0]) - math.cos(angle) * (target[1] - start[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * start[0] - start[1]
        x = abs(a * target[0] + target[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))
