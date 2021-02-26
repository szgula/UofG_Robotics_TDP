from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action
from game_interfaces.msg import Position
from Planner.src.brachistochrone import cycloid
from BasicCommonActions.go_to_action import go_to_fast


class PlayerController:  #(Robot)
    def __init__(self):
        self.read = True
        self.points_to_visit = []
        self.current_goal = 0
        self.goal_threshold = 0.1

    def get_coordinates(self,my_pos_efcs:Position, ball_pos_efcs: Position):
        x_trajectory, y_trajectory, T = cycloid(my_pos_efcs.x + 5,
                                                my_pos_efcs.y + 3,
                                                ball_pos_efcs.x + 5,
                                                ball_pos_efcs.y + 3)
        for x, y in zip(x_trajectory, y_trajectory):
            self.points_to_visit.append(Position(x, y, 0))

    def get_action(self, my_pos_efcs:Position):
        goal_pos = self.points_to_visit[self.current_goal]
        if np.hypot(goal_pos.x - my_pos_efcs.x, goal_pos.y - my_pos_efcs.y) < self.goal_threshold:
            self.current_goal += 1
            self.current_goal %= 6
        l_rpm, r_rpm, = simple_go_to_action(my_pos_efcs, goal_pos)
        print(f"({l_rpm}, {r_rpm})")
        return PlayerCommand(l_rpm, r_rpm, 0)
