import numpy as np
from game_interfaces.msg import Position, PlayerCommand, TeamPosition


class TeamMasterSupporting:
    field_size_x = 10
    field_size_y = 6
    simulation_dt = 0.1
    max_robot_speed = 0.05
    not_reachable = 1000

    @staticmethod
    def calculate_time_to_wall_collision(ball_pos, ball_vel):
        if ball_vel.x >= 0:
            dx = TeamMasterSupporting.field_size_x/2 - ball_pos.x
        else:
            dx = -TeamMasterSupporting.field_size_x/2 - ball_pos.x
        tx = dx / ball_vel.x if ball_vel.x != 0 else 1000
        if ball_vel.y >= 0:
            dy = TeamMasterSupporting.field_size_y / 2 - ball_pos.y
        else:
            dy = -TeamMasterSupporting.field_size_y / 2 - ball_pos.y
        ty = dy / ball_vel.y if ball_vel.y != 0 else 1000
        return min(tx, ty)


    @staticmethod
    def get_team_capture_time(team_pos, ball_pos_efcs, ball_vel_efcs, extra_player_time=0):
        capture_time = [0] * 5
        for player_id in range(5):
            capture_time[player_id] = TeamMasterSupporting.get_capture_time(ball_pos_efcs,
                                                            team_pos[player_id],
                                                            ball_vel_efcs,
                                                            extra_player_time)
        return capture_time

    @staticmethod
    def predict_wall_bounce(ball_pos: Position, ball_vel: Position, bounce_time: float):
        almost_zero_threshold = 0.01
        bounce_speed_damping = 0.8
        bounce_pos = TeamMasterSupporting.get_ball_pos_at_time(bounce_time, ball_pos, ball_vel)
        new_vel = Position(ball_vel.x*bounce_speed_damping, ball_vel.y*bounce_speed_damping, 0)
        if abs(abs(bounce_pos.x) - TeamMasterSupporting.field_size_x) < almost_zero_threshold:
            new_vel.x *= -1
        if abs(abs(bounce_pos.y) - TeamMasterSupporting.field_size_y) < almost_zero_threshold:
            new_vel.y *= -1
        return bounce_pos, new_vel


    @staticmethod
    def get_ball_pos_at_time(t, ball_pos: Position, ball_vel: Position) -> Position:
        x = ball_pos.x + ball_vel.x * t
        y = ball_pos.y + ball_vel.y * t
        return Position(x, y, 0)

    @staticmethod
    def get_capture_time(ball_pos, player_pos, ball_vel, extra_player_time=0):
        if abs(ball_vel.x) < 0.0001 and abs(ball_vel.y) < 0.0001:
            return TeamMasterSupporting.get_time_for_stationary_ball(ball_pos, player_pos)
        return TeamMasterSupporting.get_time_for_moving_ball(ball_pos, player_pos, ball_vel, extra_player_time)

    @staticmethod
    def get_time_for_stationary_ball(ball_pos, position):
        d = np.hypot(ball_pos.x - position.x, ball_pos.y - position.y)
        return d / TeamMasterSupporting.max_robot_speed

    @staticmethod
    def get_time_for_moving_ball(ball_pos, position, ball_vel, last_bounce_time=0):
        """
        Solve for no friction ball
        xb, yb, xb', yb', xp, yp = x ball pos, y ball pos, x ball vel, y ball bel, x player pos, y player pos
        (xb'*t + xb - xp)^2 + (yb'*t + yb - yp)^2 = max_player_vel * (t+last_bounce_time)
        """
        A = ball_vel.x
        B = ball_pos.x - position.x
        C = ball_vel.y
        D = ball_pos.y - position.y
        E = TeamMasterSupporting.max_robot_speed
        T = last_bounce_time

        a = 1 / (2 * (A ** 2 + C ** 2))
        b0 = (-2*A*B - 2*C*D + E)**2
        b1 = -4 * (A**2 + C**2)*(B**2 + D**2 - E*T)
        b = b0 + b1
        if b < 0: return TeamMasterSupporting.not_reachable
        b = b ** 0.5
        c = 2 * A * B
        d = 2 * C * D
        e = -E
        t0 = -a * (b + c + d + e)
        t1 = a * (b - c - d - e)
        if t0 < 0: t0 = TeamMasterSupporting.not_reachable
        if t1 < 0: t1 = TeamMasterSupporting.not_reachable
        return min(t0, t1) + last_bounce_time

    @staticmethod
    def get_soonest_contact(team_position: TeamPosition, opponents_position: TeamPosition):
        """
        Current limitations:
        - finds (this team) players contact time until the 3rd wall bounce (ball free moving)
        - finds opponents contact time until 1st bounce

        """
        wall_coll_time = TeamMasterSupporting.calculate_time_to_wall_collision(team_position.ball_pos_efcs,
                                                               team_position.ball_vel_efcs)
        players_capture_time = TeamMasterSupporting.get_team_capture_time(team_position.players_positions_efcs,
                                                          team_position.ball_pos_efcs,
                                                          team_position.ball_vel_efcs)
        opponents_capture_time = TeamMasterSupporting.get_team_capture_time(opponents_position.players_positions_efcs,
                                                            opponents_position.ball_pos_efcs,
                                                            opponents_position.ball_vel_efcs)

        player_id_min_time = int(np.argmin(players_capture_time))
        opponent_id_min_time = int(np.argmin(opponents_capture_time))
        team_can_get_to_ball = players_capture_time[player_id_min_time] < wall_coll_time
        opponent_can_get_to_ball = opponents_capture_time[opponent_id_min_time] < wall_coll_time
        if not team_can_get_to_ball:
            new_ball_pos = team_position.ball_pos_efcs
            new_ball_vel = opponents_position.ball_vel_efcs
            coll_time = wall_coll_time
            for _ in range(2):
                new_ball_pos, new_ball_vel = TeamMasterSupporting.predict_wall_bounce(new_ball_pos, new_ball_vel, coll_time)
                players_capture_time = TeamMasterSupporting.get_team_capture_time(team_position.players_positions_efcs,
                                                                  new_ball_pos,
                                                                  new_ball_vel,
                                                                  coll_time)
                coll_time += TeamMasterSupporting.calculate_time_to_wall_collision(new_ball_pos, new_ball_vel)
                player_id_min_time = int(np.argmin(players_capture_time))
                team_can_get_to_ball = players_capture_time[player_id_min_time] < coll_time
                if team_can_get_to_ball: break
        return team_can_get_to_ball, players_capture_time, opponent_can_get_to_ball, opponents_capture_time