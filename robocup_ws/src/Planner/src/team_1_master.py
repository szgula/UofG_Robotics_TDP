import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer, TeamMaster
from goalkeeper_controller import TempGoalkeeperController
from stay_in_place_controller import NullController
from game_interfaces.msg import Position, PlayerCommand
import numpy as np
import logging

class TeamMaster1(TeamMaster):
    def __init__(self):
        team_id = 1
        super().__init__(team_id)
        self.goalkeeper_logic = TempGoalkeeperController()
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

        self.max_robot_speed = 0.06  # ~0.06m/s = ~6cm/s -> based on the experiment

    def plan(self):
        mode, capture_pos, player_id = self.calculate_ball_contact_timing()
        super(TeamMaster1, self).plan()
        self.actions[player_id] = self.get_to_point_controller(self.team_position.players_positions_efcs[player_id],
                                                               capture_pos)


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
        wall_coll_time = self.calculate_time_to_wall_collision(self.team_position.ball_pos_efcs, self.team_position.ball_vel_efcs)
        players_capture_time = [0] * 5
        opponents_capture_time = [0] * 5
        for player_id in range(5):
            players_capture_time[player_id] = self.get_capture_time(self.team_position.ball_pos_efcs,
                                                                    self.team_position.players_positions_efcs[player_id],
                                                                    self.team_position.ball_vel_efcs)
            opponents_capture_time[player_id] = self.get_capture_time(self.opponents_position.ball_pos_efcs,
                                                                      self.opponents_position.players_positions_efcs[player_id],
                                                                      self.opponents_position.ball_vel_efcs)

        player_id_min_time = np.argmin(players_capture_time)
        opponent_id_min_time = np.argmin(opponents_capture_time)
        team_can_get_to_ball = players_capture_time[player_id_min_time] < wall_coll_time
        opponent_can_get_to_ball = opponents_capture_time[opponent_id_min_time] < wall_coll_time

        # TODO: move to separate function
        mode = "NONE"
        capture_pos = Position(0, 0, 0)
        if not team_can_get_to_ball and not opponent_can_get_to_ball:
            logging.warning("First contact can be done only after ball bounce from the wall - this is not yet handled")
            # TODO: find the ball bounce position, velocity after bounce and calculate ball capture time

        elif players_capture_time[player_id_min_time] < opponents_capture_time[opponent_id_min_time] * 1.2:
            mode = "ATTACK"
            capture_pos = self.get_ball_pos_at_time(players_capture_time[player_id_min_time],
                                                    self.team_position.ball_pos_efcs, self.team_position.ball_vel_efcs)
        else:
            mode = "DEFENCE"
        return mode, capture_pos, player_id_min_time


    @staticmethod
    def get_ball_pos_at_time(t, ball_pos: Position, ball_vel: Position) -> Position:
        x = ball_pos.x + ball_vel.x * t
        y = ball_pos.y + ball_vel.y * t
        return Position(x, y, 0)

    def get_capture_time(self, ball_pos, player_pos, ball_vel):
        if abs(ball_vel.x) < 0.0001 and abs(ball_vel.y) < 0.0001:
            return self.get_time_for_stationary_ball(ball_pos, player_pos)
        return self.get_time_for_moving_ball(ball_pos, player_pos, ball_vel)

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
        return min(t0, t1)

    @staticmethod
    def get_to_point_controller(player_pos:Position, target_pos:Position):
        dx = target_pos.x - player_pos.x
        dy = target_pos.y - player_pos.y
        d2 = dx**2 + dy**2
        target_heading = np.arctan2(dy, dx)
        player_heading = player_pos.theta
        d_heading = target_heading - player_heading
        if d_heading > np.pi: d_heading = -2*np.pi + d_heading
        elif d_heading < -np.pi: d_heading = 2*np.pi + d_heading

        if d_heading > np.deg2rad(5):
            out = PlayerCommand(0.5, -0.5, 0)
        elif d_heading < -np.deg2rad(5):
            out = PlayerCommand(-0.5, 0.5, 0)
        elif d2 > 0.05:
            out = PlayerCommand(3, 3, 0)
        else:
            out = PlayerCommand(0, 0, 0)
        return out


if __name__ == "__main__":
    team_mater_1 = TeamMaster1()
    TMS = TeamMasterServer(team_mater_1)
