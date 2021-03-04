from game_interfaces.msg import PlayerCommand
import numpy as np
from BasicCommonActions.go_to_action import simple_go_to_action, go_to_fast
from game_interfaces.msg import Position
from BasicCommonActions.plan_supporting_functions import TeamMasterSupporting


class Team1Striker:
    def __init__(self, init_y_pos, my_idx, team_id):
        self.my_team_id = team_id
        self.conversion = 1 if self.my_team_id == 0 else -1
        self.my_idx = my_idx
        self.init_y_pos = init_y_pos
        self.kick_distance = 0.17

    def get_action(self, my_pos_efcs, ball_pos_efcs, team_positions_wcs=None, opponents_positions_wcs=None):
        d_player2ball = np.hypot(my_pos_efcs.x - ball_pos_efcs.x, my_pos_efcs.y - ball_pos_efcs.y)

        opponents_x_pos_wcs = [pos.x for pos in opponents_positions_wcs]
        opponents_y_pos_wcs = [pos.y for pos in opponents_positions_wcs]

        opponents_sorted_ids_by_x = np.argsort(opponents_x_pos_wcs)
        # the last defender is second closest to net player
        last_defender_id = opponents_sorted_ids_by_x[1] if self.my_team_id == 1 else opponents_sorted_ids_by_x[-2]
        best_x_efcs = opponents_x_pos_wcs[last_defender_id] * self.conversion
        best_y_efcs = 1 * np.sign(self.init_y_pos)
        l_v, r_v = go_to_fast(my_pos_efcs, Position(best_x_efcs, best_y_efcs, 0))

        #if d_player2ball < self.kick_distance:
        #    direct_kick_feasible, kick_x, kick_y = TeamMasterSupporting.check_if_direct_goal_feasible(None, ball_pos_efcs, opponents_positions_wcs, self.my_team_id)

        return PlayerCommand(l_v, r_v, 0)



class DummyStrikerWithPredefinedPointsToMove:  #(Robot)
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

    def get_action(self, my_pos_efcs:Position, ball_pos_efcs: Position, team_positions_wcs=None, opponents_positions_wcs=None):
        goal_pos = self.points_to_visit[self.current_goal]

        if np.hypot(goal_pos.x - my_pos_efcs.x, goal_pos.y - my_pos_efcs.y) < self.goal_threshold:
            self.current_goal += 1
            self.current_goal %= 6
        l_rpm, r_rpm, = simple_go_to_action(my_pos_efcs, goal_pos)

        return PlayerCommand(l_rpm, r_rpm, 0)



class RotateController:  #(Robot)
    def __init__(self):
        pass

    def get_action(self, my_pos_efcs:Position, ball_pos_efcs: Position):
        # l = 1, r = 2: d = 0.15, r = 0.075
        # l = 1, r = 3, d = 0.1, r = 0.05

        l_rpm = 1
        r_rpm = 3
        print("[", my_pos_efcs.x, ",", my_pos_efcs.y, "],")
        return PlayerCommand(l_rpm, r_rpm, 0)