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

    def chase_ball(self, my_pos_efcs:Position, ball_pos_efcs: Position):
        lookahead = [ball_pos_efcs.x,ball_pos_efcs.y]
        pos = [my_pos_efcs.x,my_pos_efcs.y]
        angle = my_pos_efcs.theta
        curv = self.curvature(lookahead, pos, angle)
        width = 0.2
        wheels = [(2 + curv * width) / 2, (2 - curv * width) / 2]
        print(f"({wheels[1]}, {wheels[0]})")
        return PlayerCommand(wheels[1], wheels[0], 0)

    def curvature(self, lookahead, pos, angle):
        side = np.sign(math.sin(angle) * (lookahead[0] - pos[0]) - math.cos(angle) * (lookahead[1] - pos[1]))
        a = -math.tan(angle)
        c = math.tan(angle) * pos[0] - pos[1]
        x = abs(a * lookahead[0] + lookahead[1] + c) / math.sqrt(a ** 2 + 1)
        return side * (2 * x / (float(1) ** 2))
