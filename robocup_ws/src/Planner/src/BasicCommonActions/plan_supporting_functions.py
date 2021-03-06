import numpy as np
from game_interfaces.msg import Position, PlayerCommand, TeamPosition
import matplotlib.pyplot as plt
from typing import List


class TeamMasterSupporting:
    field_size_x = 10
    field_size_y = 6
    net_length = 2
    simulation_dt = 0.1
    max_robot_speed = 0.05
    max_robot_speed_optimistic = 0.1
    not_reachable = 1000
    kick_speed = 0.4

    @staticmethod
    def calculate_time_to_wall_collision(ball_pos: Position, ball_vel: Position):
        """
        Returns estimated nearest time when the ball will hit the wall
        It assumes no players interactions - the ball is free to move

        Returns: float, time
        """
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
    def get_team_capture_time(team_pos: List[Position], ball_pos_efcs: Position, ball_vel_efcs: Position, extra_player_time: float = 0):
        """
        Returns an list of times:
        Each cell include an estimated time within which the robot can get to the ball.
        It assumes no ball friction
        It assumes infinitely large field

        Returns list[5], times for each team member
        """
        capture_time = [0] * 5
        for player_id in range(5):
            capture_time[player_id] = TeamMasterSupporting.get_capture_time(ball_pos_efcs,
                                                            team_pos[player_id],
                                                            ball_vel_efcs,
                                                            extra_player_time)
        return capture_time

    @staticmethod
    def predict_wall_bounce(ball_pos: Position, ball_vel: Position, bounce_time: float):
        """
        If no robot can get to ball before it bounce the wall, we estimate the ball bounce position and
        ball velocity after the bounce
        This enable us to find intersection point after the bounce

        Returns: bounce_pos (Position) - the location of the bounce
        Returns: new_vel (Position) - estimated velocity after the bounce
        """
        almost_zero_threshold = 0.01
        bounce_speed_damping = 0.8
        bounce_pos = TeamMasterSupporting.get_ball_pos_at_time(bounce_time, ball_pos, ball_vel)
        new_vel = Position(ball_vel.x*bounce_speed_damping, ball_vel.y*bounce_speed_damping, 0)
        if abs(abs(bounce_pos.x) - TeamMasterSupporting.field_size_x / 2) <= almost_zero_threshold:
            new_vel.x *= -1
        if abs(abs(bounce_pos.y) - TeamMasterSupporting.field_size_y / 2) <= almost_zero_threshold:
            new_vel.y *= -1
        return bounce_pos, new_vel

    @staticmethod
    def get_ball_pos_at_time(t: float, ball_pos: Position, ball_vel: Position) -> Position:
        """
        Estimate ball position at given time
        It assumes the ball will not interact with any players within given time

        TODO: write testcases to verify if
        """
        new_pos, new_vel = ball_pos, ball_vel
        first_collision_time = TeamMasterSupporting.calculate_time_to_wall_collision(ball_pos, ball_vel)
        if t > first_collision_time:
            new_pos, new_vel = TeamMasterSupporting.predict_wall_bounce(ball_pos, ball_vel, first_collision_time)
            # second_collision_time - time between first bounce and second bounce
            second_collision_time = TeamMasterSupporting.calculate_time_to_wall_collision(new_pos, new_vel)
            t -= first_collision_time
            if t > second_collision_time:
                new_pos, new_vel = TeamMasterSupporting.predict_wall_bounce(new_pos, new_vel, second_collision_time)
                t -= second_collision_time

        x = new_pos.x + new_vel.x * t
        y = new_pos.y + new_vel.y * t
        return Position(x, y, 0)

    @staticmethod
    def get_capture_time(ball_pos: Position, player_pos: Position, ball_vel: Position, extra_player_time: float = 0):
        """
        Estimates the time in which a player can get to the ball

        Return: float, time
        """
        if abs(ball_vel.x) < 0.0001 and abs(ball_vel.y) < 0.0001:
            return TeamMasterSupporting.get_time_for_stationary_ball(ball_pos, player_pos)
        return TeamMasterSupporting.get_time_for_moving_ball(ball_pos, player_pos, ball_vel, extra_player_time)

    @staticmethod
    def get_time_for_stationary_ball(ball_pos: Position, position: Position):
        """
        Estimates the time in which a player can get to the stationary ball

        Return: float, time
        """
        d = np.hypot(ball_pos.x - position.x, ball_pos.y - position.y)
        return d / TeamMasterSupporting.max_robot_speed

    @staticmethod
    def get_time_for_moving_ball(ball_pos, position, ball_vel, last_bounce_time=0):
        """
        Estimates the time in which a player can get to the moving ball
        It assumes the infinitely large field

        Return: float, time

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
        Estimate time when all players can get to the ball
        Check if players can get to ball within given time horizon
        Opponents horizon: first ball bounce from wall
        Players horizon: 3rd ball bounce from wall

        Returns: Bool, team_can_get_to_ball: flag if our team can get to ball within given horizon
        Returns: list[5], players_capture_time: list of times when players can get to ball
        Returns: Bool, opponent_can_get_to_ball: flag if opponent's team can get to ball within given horizon
        Returns: list[5], opponents_capture_time: list of times when opponents can get to ball

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
            new_ball_vel = team_position.ball_vel_efcs
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
        """
        Returns true if robot has enough time to go around ball and prepare to kick in any direction
        """
        # TODO: improve this logic
        min_rotation_time = 2  # TODO: experiment needed to find the value
        all_safe = all(np.arrya(opponents_capture_time) > min_rotation_time)
        return all_safe or (not opponent_can_get_to_ball)

    @staticmethod
    def find_safe_players_to_pass(team_position: TeamPosition, opponents_position: TeamPosition, team_id, player_with_ball_id, vis=False):
        """
        Check which players are safe to pass the ball to
        """
        players_pos_wcs = np.array([[pos.x, pos.y] for pos in team_position.players_positions_wcs])
        opponents_pos_wcs = np.array([[pos.x, pos.y] for pos in opponents_position.players_positions_wcs])
        ball_pos_wcs = team_position.ball_pos_efcs if team_id == 0 else opponents_position.ball_pos_efcs
        ball_pos_wcs = np.array([ball_pos_wcs.x, ball_pos_wcs.y])

        d = ball_pos_wcs - players_pos_wcs
        slope = d[:, 1] / d[:, 0]
        # FIXME: slope = 0, prevent 0 division -> this results in inf and is handled later on
        slope_perpendicular = -1 / slope

        points, points_vis = [], []
        for i in range(5):
            s = slope_perpendicular[i]
            p = players_pos_wcs[i]
            a, a_vis = TeamMasterSupporting.get_intersection_region(ball_pos_wcs, p, s)
            points.append(a)
            points_vis.append(a_vis)

        if vis:  # This is just for debug visualisation
            pos_opponent = [
                (opponents_position.players_positions_wcs[i].x, opponents_position.players_positions_wcs[i].y)
                for i in range(5)]
            pos_opponent = np.array(pos_opponent)
            TeamMasterSupporting.debug_visualize_pass_danger_zones(points_vis, pos_opponent)

        # TODO: check if any opponent in "pass danger zone" - opponent able to capture the ball
        # TODO: if opponent in "danger zone" - find alternative kick angles (not direct ones) to safely pass the ball
        # TODO: select the best player to pass ball to
        best_player_to_pass = 2
        return points, best_player_to_pass

    @staticmethod
    def debug_visualize_pass_danger_zones(points_vis, pos_opponent):
        """
        Visualise danger zones - for debugging porpoises
        """
        for a in points_vis:
            plt.plot(a[:, 0], a[:, 1])
        pos_opponent = np.array(pos_opponent)
        plt.scatter(pos_opponent[:, 0], pos_opponent[:, 1], s=100)

    @staticmethod
    def get_intersection_region(ball_position: np.array, player_position: np.array, kick_slope: float):
        """
        Estimate the region from where an opponent can capture the ball
        The region is represented by a triangle

        Returns characteristic points of "pass danger zone":
        a = [danger_zone_corner_0 (ball_position),
            player_to_pass_position (midpoint between corner 1 and 2)
            danger_zone_corner_1,
            danger_zone_corner_2]

        a_vis - is visualization array for pyplot (just for debugging)

        """
        d = np.hypot(*(ball_position-player_position)) * TeamMasterSupporting.max_robot_speed_optimistic
        d /= TeamMasterSupporting.kick_speed
        n = TeamMasterSupporting.get_point_on_vector(player_position, kick_slope, d)
        n_ = TeamMasterSupporting.get_point_on_vector(player_position, kick_slope, -d)
        a = np.array([ball_position, player_position, n, n_])
        a_vis = np.array([ball_position, n, n_, ball_position, player_position])  # a[:, 0] - x coord, a[:, 1] - y coord
        return a, a_vis

    @staticmethod
    def get_point_on_vector(initial_pt: np.array, slope: float, distance:float):
        """
        Returns a point in given distance from the initial point that lies on the line with a given slope

        This is supporting math function

        Returns (x, y) coordinate
        """
        if slope == np.inf or slope == -np.inf:
            return (initial_pt[0], initial_pt[1] + distance)
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
    def lineseg_dists(p, a, b):
        """
        Math supporting function, calculate distance between point and line segment

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

    @staticmethod
    def check_if_direct_goal_feasible(player_pos_wcs: Position, ball_pos: Position, opponents_pos_wcs: List[Position], team_id: int, vis: bool = False):
        """
        Check if from current position it is possible to kick a goal
        It takes into consideration opponents position

        Returns: direct_kick_feasible, bool - flag if direct goal is possible
        Returns: kick_x, float - x coordinate where the goal is possible
        Returns: kick_y, float - y coordinate where the goal is possible
        """
        points_to_check = 5
        conversion = 1 if team_id == 0 else -1
        ball_pos_wcs = np.array([ball_pos.x * conversion, ball_pos.y * conversion])
        opponents_wcs = np.array([[pos.x, pos.y] for pos in opponents_pos_wcs])
        net_pos_x_wcs = 5 if team_id == 0 else -5
        net_pos_y_0 = TeamMasterSupporting.net_length / 2 - 0.1
        net_pos_y_1 = -TeamMasterSupporting.net_length / 2 + 0.1

        x = np.array([net_pos_x_wcs for _ in range(points_to_check)])
        distance_between_points = (net_pos_y_0 - net_pos_y_1) / (points_to_check - 1)
        y = np.array([net_pos_y_1 + distance_between_points * d for d in range(points_to_check)])
        slope = y / x

        slope_perpendicular = -1 / slope
        points_wcs, points_vis = [], []
        for i in range(5):
            s = slope_perpendicular[i]
            p = np.array([x[i], y[i]])
            a, a_vis = TeamMasterSupporting.get_intersection_region(ball_pos_wcs, p, s)
            points_wcs.append(a)
            points_vis.append(a_vis)

        if vis:  # This is just for debug visualisation
            TeamMasterSupporting.debug_visualize_pass_danger_zones(points_vis, opponents_wcs)

        free_to_kick = []
        for zone in points_wcs:
            no_enemies = True
            for opp in opponents_wcs:
                no_enemies &= not(TeamMasterSupporting.point_in_triangle(opp, zone[0], zone[2], zone[3]))
                if not no_enemies:
                    break
            free_to_kick.append(no_enemies)

        # Select point to kick
        direct_kick_feasible = False
        kick_x, kick_y = None, None
        if any(free_to_kick):
            for idx, status in enumerate(free_to_kick):
                if status:
                    break
            direct_kick_feasible = True
            kick_x = x[idx] * conversion
            kick_y = y[idx] * conversion

        return direct_kick_feasible, kick_x, kick_y

    @staticmethod
    def triangle_area(p1, p2, p3):
        """
        Supporting math function: calculate triangle position
        This is used to check if opponent is within danger zone
        """

        return abs((p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])) / 2.0)

    @staticmethod
    def point_in_triangle(point, t0, t1, t2):
        """
        Supporting math function: check if point is within triangle (t0, t1, t2)
        This is used to check if opponent is within danger zone
        https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
        """
        # Calculate area of triangle ABC
        A = TeamMasterSupporting.triangle_area(t0, t1, t2)
        A1 = TeamMasterSupporting.triangle_area(point, t1, t2)
        A2 = TeamMasterSupporting.triangle_area(t0, point, t2)
        A3 = TeamMasterSupporting.triangle_area(t0, t1, point)
        return abs(A - (A1 + A2 + A3)) < 0.01

