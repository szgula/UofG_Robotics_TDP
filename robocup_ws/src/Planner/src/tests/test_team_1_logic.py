import numpy as np
import matplotlib.pyplot as plt
from game_interfaces.msg import Position

class TeamMaster1:
    def __init__(self):

        ############### PLAYERS INTERNAL ############
        self.field_size_x = 10
        self.field_size_y = 6
        self.field_grid_scale = 10
        self.x_field_resolution = self.field_size_x * self.field_grid_scale + 1
        self.y_field_resolution = self.field_size_y * self.field_grid_scale + 1
        self.simulation_dt = 0.1
        self.ball_distance_grid = np.zeros((self.x_field_resolution, self.y_field_resolution))

        self.max_robot_speed = 0.06  # ~0.06m/s = ~6cm/s -> based on the experiment
        self.not_reachable = 0

    def calculate_ball_distance_grid(self):

        ball_pos = Position(0, 0, 0)
        ball_vel = Position(0.02, 0.1, 0)

        for ix in range(self.x_field_resolution):
            for iy in range(self.y_field_resolution):
                x_pos = (ix / self.field_grid_scale) - self.field_size_x / 2
                y_pos = (iy / self.field_grid_scale) - self.field_size_y / 2
                pos = Position(x_pos, y_pos, 0)
                self.ball_distance_grid[ix, iy] = self.get_time_for_moving_ball(ball_pos, pos, ball_vel)
                #self.ball_distance_grid[ix, iy] = self.get_time_for_stationary_ball(ball_pos, pos)


        np.place(self.ball_distance_grid, self.ball_distance_grid == 0,  np.max(self.ball_distance_grid) * 1.2)
        plt.imshow(self.ball_distance_grid.T, cmap='coolwarm', interpolation='nearest')
        plt.colorbar()
        plt.show()



    def get_radius_for_time_step(self, i):
        return i * self.simulation_dt * self.max_robot_speed

    def get_grid_idx_for_pos(self, pos_x, pos_y):
        x_idx = np.floor((pos_x + self.field_size_x / 2) * self.field_grid_scale)
        y_idy = np.floor((pos_y + self.field_size_y / 2) * self.field_grid_scale)
        return x_idx, y_idy

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
        # b0 = -4 * A ** 2 * D ** 2
        # b1 = 8 * A * B * C * D
        # b2 = -4 * A * B * E
        # b3 = -4 * B ** 2 * C ** 2
        # b4 = -4 * C * D * E
        # b5 = E ** 2
        # b = (b0 + b1 + b2 + b3 + b4 + b5)
        b0 = (-2 * A * B - 2 * C * D + E) ** 2
        b1 = -4 * (A ** 2 + C ** 2) * (B ** 2 + D ** 2 - E * T)
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


if __name__ == "__main__":
    temp = TeamMaster1()
    temp.calculate_ball_distance_grid()
    pass
