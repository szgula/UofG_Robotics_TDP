import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer, TeamMaster
from goalkeeper_controller import Team1GoalkeeperController
from striker_controller import Team1StrikerController
from stay_in_place_controller import NullController
from game_interfaces.msg import Position, PlayerCommand
from BasicCommonActions.go_to_action import go_to_fast
import numpy as np
import logging


class TeamMaster1(TeamMaster):
    def __init__(self):
        team_id = 1
        super().__init__(team_id)
        self.goalkeeper_logic = Team1GoalkeeperController(1)
        self.striker_left_logic = NullController()
        self.striker_right_logic = NullController()
        self.defence_left_logic = NullController()
        self.defence_right_logic = NullController()
        self.players_logic_was_updated = True

        ############### PLAYERS INTERNAL ############
        self.field_size_x = 10
        self.field_size_y = 6
        self.field_grid_scale = 10
        self.x_field_resolution = self.field_size_x * self.field_grid_scale + 1
        self.y_field_resolution = self.field_size_y * self.field_grid_scale + 1
        self.simulation_dt = 0.1
        self.ball_distance_grid = np.zeros((self.x_field_resolution, self.y_field_resolution))

        self.max_robot_speed = 0.05  # ~0.06m/s = ~6cm/s -> based on the experiment
        self.not_reachable = 1000

    def plan(self):
        mode, capture_pos, player_id = self.calculate_ball_contact_timing()
        player_id = 3
        super(TeamMaster1, self).plan()
        if mode == "ATTACK" or True:
            self.actions[player_id] = self.simple_go_to_point(self.team_position.players_positions_efcs[player_id],
                                                              capture_pos, self.team_position.ball_pos_efcs)

    def calculate_time_to_wall_collision(self, ball_pos, ball_vel):
        if ball_vel.x >= 0:
            dx = self.field_size_x/2 - ball_pos.x
        else:
            dx = -self.field_size_x/2 - ball_pos.x
        tx = dx / ball_vel.x if ball_vel.x != 0 else 1000
        if ball_vel.y >= 0:
            dy = self.field_size_y / 2 - ball_pos.y
        else:
            dy = -self.field_size_y / 2 - ball_pos.y
        ty = dy / ball_vel.y if ball_vel.y != 0 else 1000
        return min(tx, ty)

    def calculate_ball_contact_timing(self):
        team_can_get_to_ball, players_capture_time, opponent_can_get_to_ball, opponents_capture_time = self.get_soonest_contact()
        player_id_min_time = np.argmin(players_capture_time)
        opponent_id_min_time = np.argmin(opponents_capture_time)
        mode = "NONE"
        capture_pos = Position(0, 0, 0)
        if not team_can_get_to_ball and not opponent_can_get_to_ball:
            logging.warning("First contact can be done only after ball bounce from the wall - this is not yet handled")
            # TODO: find the ball bounce position, velocity after bounce and calculate ball capture time

        #elif players_capture_time[player_id_min_time] < opponents_capture_time[opponent_id_min_time] * 1.2:
        else:
            mode = "ATTACK"
            capture_pos = self.get_ball_pos_at_time(players_capture_time[player_id_min_time],
                                                    self.team_position.ball_pos_efcs, self.team_position.ball_vel_efcs)
        return mode, capture_pos, player_id_min_time

    def get_soonest_contact(self):
        wall_coll_time = self.calculate_time_to_wall_collision(self.team_position.ball_pos_efcs,
                                                               self.team_position.ball_vel_efcs)
        players_capture_time = self.get_team_capture_time(self.team_position.players_positions_efcs,
                                                          self.team_position.ball_pos_efcs,
                                                          self.team_position.ball_vel_efcs)
        opponents_capture_time = self.get_team_capture_time(self.opponents_position.players_positions_efcs,
                                                            self.opponents_position.ball_pos_efcs,
                                                            self.opponents_position.ball_vel_efcs)

        player_id_min_time = np.argmin(players_capture_time)
        opponent_id_min_time = np.argmin(opponents_capture_time)
        team_can_get_to_ball = players_capture_time[player_id_min_time] < wall_coll_time
        opponent_can_get_to_ball = opponents_capture_time[opponent_id_min_time] < wall_coll_time
        if not team_can_get_to_ball:
            new_ball_pos = self.team_position.ball_pos_efcs
            new_ball_vel = self.opponents_position.ball_vel_efcs
            coll_time = wall_coll_time
            for _ in range(2):
                new_ball_pos, new_ball_vel = self.predict_wall_bounce(new_ball_pos, new_ball_vel, coll_time)
                players_capture_time = self.get_team_capture_time(self.team_position.players_positions_efcs,
                                                                  new_ball_pos,
                                                                  new_ball_vel,
                                                                  coll_time)
                coll_time += self.calculate_time_to_wall_collision(new_ball_pos, new_ball_vel)
                player_id_min_time = np.argmin(players_capture_time)
                team_can_get_to_ball = players_capture_time[player_id_min_time] < coll_time
                if team_can_get_to_ball: break
        return team_can_get_to_ball, players_capture_time, opponent_can_get_to_ball, opponents_capture_time


    def get_team_capture_time(self, team_pos, ball_pos_efcs, ball_vel_efcs, extra_player_time=0):
        capture_time = [0] * 5
        for player_id in range(5):
            capture_time[player_id] = self.get_capture_time(ball_pos_efcs,
                                                            team_pos[player_id],
                                                            ball_vel_efcs,
                                                            extra_player_time)
        return capture_time

    def predict_wall_bounce(self, ball_pos: Position, ball_vel: Position, bounce_time: float):
        almost_zero_threshold = 0.01
        bounce_speed_damping = 0.8
        bounce_pos = self.get_ball_pos_at_time(bounce_time, ball_pos, ball_vel)
        new_vel = Position(ball_vel.x*bounce_speed_damping, ball_vel.y*bounce_speed_damping, 0)
        if abs(abs(bounce_pos.x) - self.field_size_x) < almost_zero_threshold:
            new_vel.x *= -1
        if abs(abs(bounce_pos.y) - self.field_size_y) < almost_zero_threshold:
            new_vel.y *= -1
        return bounce_pos, new_vel


    @staticmethod
    def get_ball_pos_at_time(t, ball_pos: Position, ball_vel: Position) -> Position:
        x = ball_pos.x + ball_vel.x * t
        y = ball_pos.y + ball_vel.y * t
        return Position(x, y, 0)

    def get_capture_time(self, ball_pos, player_pos, ball_vel, extra_player_time=0):
        if abs(ball_vel.x) < 0.0001 and abs(ball_vel.y) < 0.0001:
            return self.get_time_for_stationary_ball(ball_pos, player_pos)
        return self.get_time_for_moving_ball(ball_pos, player_pos, ball_vel, extra_player_time)

    def get_time_for_stationary_ball(self, ball_pos, position):
        d = np.hypot(ball_pos.x - position.x, ball_pos.y - position.y)
        return d / self.max_robot_speed

    def get_time_for_moving_ball(self, ball_pos, position, ball_vel, last_bounce_time=0):
        """
        Solve for no friction ball
        xb, yb, xb', yb', xp, yp = x ball pos, y ball pos, x ball vel, y ball bel, x player pos, y player pos
        (xb'*t + xb - xp)^2 + (yb'*t + yb - yp)^2 = max_player_vel * t
        """
        A = ball_vel.x
        B = ball_pos.x - position.x
        C = ball_vel.y
        D = ball_pos.y - position.y
        E = self.max_robot_speed
        T = last_bounce_time

        a = 1 / (2 * (A ** 2 + C ** 2))
        #b0 = -4 * A ** 2 * D ** 2
        #b1 = 8 * A * B * C * D
        #b2 = -4 * A * B * E
        #b3 = -4 * B ** 2 * C ** 2
        #b4 = -4 * C * D * E
        #b5 = E ** 2
        #b = (b0 + b1 + b2 + b3 + b4 + b5)
        b0 = (-2*A*B - 2*C*D + E)**2
        b1 = -4 * (A**2 + C**2)*(B**2 + D**2 - E*T)
        b = b0 + b1
        if b < 0: return self.not_reachable
        b = b ** 0.5
        c = 2 * A * B
        d = 2 * C * D
        e = -E
        t0 = -a * (b + c + d + e)
        t1 = a * (b - c - d - e)
        if t0 < 0: t0 = self.not_reachable
        if t1 < 0: t1 = self.not_reachable
        return min(t0, t1) + last_bounce_time

    def simple_go_to_point(self, robot_state: Position, target: Position, ball_position: Position):
        d = np.hypot(robot_state.x - ball_position.x, robot_state.y - ball_position.y)
        action = 0
        if d < 0.15:
            action = 1
        return PlayerCommand(*go_to_fast(robot_state, target), action)





if __name__ == "__main__":
    team_mater_1 = TeamMaster1()
    TMS = TeamMasterServer(team_mater_1)
