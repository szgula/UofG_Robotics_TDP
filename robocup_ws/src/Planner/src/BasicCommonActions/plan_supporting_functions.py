import numpy as np
from game_interfaces.msg import Position, PlayerCommand, TeamPosition
import matplotlib.pyplot as plt


class TeamMasterSupporting:
    field_size_x = 10
    field_size_y = 6
    simulation_dt = 0.1
    max_robot_speed = 0.05
    max_robot_speed_optimistic = 0.07
    not_reachable = 1000
    kick_speed = 0.4

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

    @staticmethod
    def is_safe_to_pass_anywhere(opponent_can_get_to_ball, opponents_capture_time):
        min_rotation_time = 2
        all_safe = all(np.arrya(opponents_capture_time) > min_rotation_time)
        return all_safe or (not opponent_can_get_to_ball)

    @staticmethod
    def find_safe_players_to_pass(team_position: TeamPosition, opponents_position: TeamPosition, team_id, vis=False):
        players_pos_wcs = np.array([[pos.x, pos.y] for pos in team_position.players_positions_wcs])
        opponents_pos_wcs = np.array([[pos.x, pos.y] for pos in opponents_position.players_positions_wcs])
        ball_pos_wcs = team_position.ball_pos_efcs if team_id == 0 else opponents_position.ball_pos_efcs
        ball_pos_wcs = np.array([ball_pos_wcs.x, ball_pos_wcs.y])

        d = ball_pos_wcs - players_pos_wcs
        slope = d[:, 1] / d[:, 0]
        # FIXME: slope = 0
        slope_perpendicular = -1 / slope

        points, points_vis = [], []
        for i in range(5):
            s = slope_perpendicular[i]
            p = players_pos_wcs[i]
            a, a_vis = TeamMasterSupporting.get_intersection_region(ball_pos_wcs, p, s)
            points.append(a)
            points_vis.append(a_vis)
        if vis:
            for a in points_vis:
                plt.plot(a[:, 0], a[:, 1])
            pos_opponent = [
                (opponents_position.players_positions_wcs[i].x, opponents_position.players_positions_wcs[i].y)
                for i in range(5)]
            pos_opponent = np.array(pos_opponent)
            plt.scatter(pos_opponent[:, 0], pos_opponent[:, 1], s=100)
        best_player_to_pass = 2
        return points, best_player_to_pass

    @staticmethod
    def get_intersection_region(ball_position, player_position, kick_slope):
        d = np.hypot(*(ball_position-player_position)) * TeamMasterSupporting.max_robot_speed_optimistic
        d /= TeamMasterSupporting.kick_speed
        n = TeamMasterSupporting.get_point_on_vector(player_position, kick_slope, d)
        n_ = TeamMasterSupporting.get_point_on_vector(player_position, kick_slope, -d)
        a = np.array([ball_position, player_position, n, n_])
        a_vis = np.array([ball_position, n, n_, ball_position, player_position])  # a[:, 0] - x coord, a[:, 1] - y coord
        return a, a_vis

    @staticmethod
    def get_point_on_vector(initial_pt, slope, distance):
        dx = 1
        dy = slope * dx
        terminal_pt = [initial_pt[0] + dx, initial_pt[1] + dy]
        v = np.array(initial_pt, dtype=float)
        u = np.array(terminal_pt, dtype=float)
        n = v - u
        n /= np.linalg.norm(n, 2)
        point = v - distance * n

        return tuple(point)

    @staticmethod
    def test_generate_new_point():
        pass



    @staticmethod
    def lineseg_dists(p, a, b):
        """
        https://stackoverflow.com/a/58781995
        Cartesian distance from point to line segment

        Edited to support arguments as series, from:
        https://stackoverflow.com/a/54442561/11208892

        Args:
            - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
            - a: np.array of shape (x, 2)
            - b: np.array of shape (x, 2)
        """
        # normalized tangent vectors
        d_ba = b - a
        d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1])
                             .reshape(-1, 1)))

        # signed parallel distance components
        # rowwise dot products of 2D vectors
        s = np.multiply(a - p, d).sum(axis=1)
        t = np.multiply(p - b, d).sum(axis=1)

        # clamped parallel distance
        h = np.maximum.reduce([s, t, np.zeros(len(s))])

        # perpendicular distance component
        # rowwise cross products of 2D vectors
        d_pa = p - a
        c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

        return np.hypot(h, c)