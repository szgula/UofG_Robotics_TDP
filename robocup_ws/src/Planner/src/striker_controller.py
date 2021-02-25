from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action
from game_interfaces.msg import Position


class Team1StrikerController:  #(Robot)
    def __init__(self):
        self.read = True
        self.points_to_visit = [Position(1,1, 0),
                                Position(-1, -1, 0),
                                Position(2, 0, 0),
                                Position(0, -2, 0),
                                Position(-5, 2, 0),
                                Position(5, -3, 0)]
        self.current_goal = 0
        self.goal_threshold = 0.1

    def get_action(self, my_pos_efcs:Position, ball_pos_efcs: Position):
        goal_pos = self.points_to_visit[self.current_goal]

        if np.hypot(goal_pos.x - my_pos_efcs.x, goal_pos.y - my_pos_efcs.y) < self.goal_threshold:
            self.current_goal += 1
            self.current_goal %= 6
        l_rpm, r_rpm, = simple_go_to_action(my_pos_efcs, goal_pos)

        return PlayerCommand(l_rpm, r_rpm, 0)
