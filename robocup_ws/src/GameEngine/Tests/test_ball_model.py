class BallTests:

    @staticmethod
    def test_ball_stationary():
        import matplotlib.pyplot as plt
        x0, y0 = 0, 0
        steps = 1000
        ball = BallBasicModel(init_x_pos=x0, init_y_pos=y0)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(steps):
            ball.step()
            x, y = ball.get_position()
            history['x'].append(x)
            history['y'].append(y)
            history['t'].append(i)
        plt.plot(history['t'], history['x'], label='x_pos')
        plt.plot(history['t'], history['y'], label='y_pos')
        return True

    @staticmethod
    def test_ball_moving_with_initial_speed(vx0=1, vy0=-2):
        import matplotlib.pyplot as plt
        x0, y0 = 0, 0
        steps = 1000
        ball = BallBasicModel(init_x_pos=x0, init_y_pos=y0)
        ball._x_vel, ball._y_vel = vx0, vy0
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(steps):
            ball.step()
            x, y = ball.get_position()
            history['x'].append(x), history['y'].append(y), history['t'].append(i)
            history['dx'].append(ball._x_vel), history['dy'].append(ball._y_vel)
        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(history['x'], history['y'], label='pos')
        ax1.legend()
        ax2.plot(history['t'], history['x'], label='x_pos(t)')
        ax2.plot(history['t'], history['y'], label='y_pos(t)')
        ax2.legend()
        ax3.plot(history['t'], history['dx'], label='x_vel(t)')
        ax3.plot(history['t'], history['dy'], label='y_vel(t)')
        ax3.legend()
        return True

    @staticmethod
    def test_ball_wall_bouncing(wall_type=CollisionTypes.WALL_VERTICAL):
        import matplotlib.pyplot as plt
        x0, y0 = 0, 0
        vx0, vy0 = 1, 2
        steps, wall_step = 1000, 300
        ball = BallBasicModel(init_x_pos=x0, init_y_pos=y0)
        ball._x_vel, ball._y_vel = vx0, vy0
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(steps):
            ball.step()
            if i == wall_step: ball._wall_collision(wall_type)
            x, y = ball.get_position()
            history['x'].append(x), history['y'].append(y), history['t'].append(i)
            history['dx'].append(ball._x_vel), history['dy'].append(ball._y_vel)

        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(history['x'], history['y'], label='pos')
        ax1.legend()
        ax2.plot(history['t'], history['x'], label='x_pos(t)')
        ax2.plot(history['t'], history['y'], label='y_pos(t)')
        ax2.legend()
        ax3.plot(history['t'], history['dx'], label='x_vel(t)')
        ax3.plot(history['t'], history['dy'], label='y_vel(t)')
        ax3.legend()
        return True

    @staticmethod
    def test_kick(player_pos_x, player_pos_y):
        import matplotlib.pyplot as plt

        class DummpPlayer:
            def __init__(self, x, y):
                self.x, self.y = x, y

            def get_position_components_wcs(self):
                return self.x, self.y

        x0, y0 = 0, 0
        player = DummpPlayer(player_pos_x, player_pos_y)
        steps, kick_step, kick_speed = 1000, 200, 5
        ball = BallBasicModel(init_x_pos=x0, init_y_pos=y0)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(steps):
            ball.step()
            if i == kick_step: ball._kick_collision(player, kick_speed)
            x, y = ball.get_position()
            history['x'].append(x), history['y'].append(y), history['t'].append(i)
            history['dx'].append(ball._x_vel), history['dy'].append(ball._y_vel)

        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(history['x'], history['y'], label='pos')
        ax1.legend()
        ax2.plot(history['t'], history['x'], label='x_pos(t)')
        ax2.plot(history['t'], history['y'], label='y_pos(t)')
        ax2.legend()
        ax3.plot(history['t'], history['dx'], label='x_vel(t)')
        ax3.plot(history['t'], history['dy'], label='y_vel(t)')
        ax3.legend()
        return True

    @staticmethod
    def test_receive(player_vel_x=0, player_vel_y=0):
        import matplotlib.pyplot as plt
        class DummpPlayer:
            def __init__(self, x, y):
                self.x, self.y = x, y

            def get_velocity_components_wcs(self):
                return self.x, self.y

        x0, y0, vx0, vy0 = 0, 0, 5, 3
        player = DummpPlayer(player_vel_x, player_vel_y)
        steps, kick_step = 1000, 200
        ball = BallBasicModel(init_x_pos=x0, init_y_pos=y0)
        ball._x_vel, ball._y_vel = vx0, vy0
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(steps):
            ball.step()
            if i == kick_step: ball._receive_collision(player)
            x, y = ball.get_position()
            history['x'].append(x), history['y'].append(y), history['t'].append(i)
            history['dx'].append(ball._x_vel), history['dy'].append(ball._y_vel)

        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(history['x'], history['y'], label='pos')
        ax1.legend()
        ax2.plot(history['t'], history['x'], label='x_pos(t)')
        ax2.plot(history['t'], history['y'], label='y_pos(t)')
        ax2.legend()
        ax3.plot(history['t'], history['dx'], label='x_vel(t)')
        ax3.plot(history['t'], history['dy'], label='y_vel(t)')
        ax3.legend()
        return True

    @staticmethod
    def test_collision_with_round_player(player_pos_x=0, player_pos_y=10,
                                         player_vel_x=0, player_vel_y=0,
                                         ball_pos_x=0, ball_pos_y=0,
                                         ball_vel_x=0, ball_vel_y=5,
                                         player_radius=0.1, dt=0.01, time=100):
        import matplotlib.pyplot as plt

        class DummpPlayer:
            def __init__(self, x, y, vx, vy, r):
                self.x, self.y = x, y
                self.vx, self.vy = vx, vy
                self.r_sq = r*r
                self.radius = r

            def get_velocity_components_wcs(self):
                return self.vx, self.vy

            def get_position_components_wcs(self):
                return self.x, self.y

            def in_range(self, x, y):
                diff = (x-self.x)**2 + (y-self.y)**2
                return diff <= self.r_sq * 4 # add some threshold for leg or sth

        player = DummpPlayer(player_pos_x, player_pos_y, player_vel_x, player_vel_y, player_radius)
        steps, event_index = int(time/dt), int(0.5*time/dt)
        ball = BallBasicModel(init_x_pos=ball_pos_x, init_y_pos=ball_pos_y, dt=dt)
        ball._x_vel, ball._y_vel = ball_vel_x, ball_vel_y
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}
        collide = False
        for i in range(steps):
            ball.step()
            if player.in_range(*ball.get_position()) and not collide:
                ball._elastic_collision_with_round_player(player)
                collide = True
            x, y = ball.get_position()
            history['x'].append(x), history['y'].append(y), history['t'].append(i*dt)
            history['dx'].append(ball._x_vel), history['dy'].append(ball._y_vel)

        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        f.suptitle(f"test_collision_with_round_player: initial conditions: \nplayer (x, y, x', y', size): {player_pos_x, player_pos_y,player_vel_x, player_vel_y, player_radius} \nball (x, y, x', y'): {ball_pos_x, ball_pos_y,ball_vel_x, ball_vel_y}, \ndt={dt}, time={time} ")
        ax1.plot(history['x'], history['y'], label='pos')
        ax1.scatter(player_pos_x, player_pos_y, label='player')  # TODO -> add marker size same as the player size
        ax1.set_xlabel('x'), ax1.set_xlabel('y')
        ax1.legend()
        ax2.plot(history['t'], history['x'], label='x_pos(t)')
        ax2.plot(history['t'], history['y'], label='y_pos(t)')
        ax2.set_xlabel('t'), ax1.set_xlabel('x & y')
        ax2.legend()
        ax3.plot(history['t'], history['dx'], label='x_vel(t)')
        ax3.plot(history['t'], history['dy'], label='y_vel(t)')
        ax3.set_xlabel('t'), ax1.set_xlabel('x & y')
        ax3.legend()
        return True


if __name__ == "__main__":
    #BallTests.test_ball_stationary()
    #BallTests.test_ball_moving_with_initial_speed(2,1)
    #BallTests.test_ball_wall_bouncing(CollisionTypes.WALL_VERTICAL)
    BallTests.test_ball_wall_bouncing(CollisionTypes.WALL_HORIZONTAL)
    #BallTests.test_ball_wall_bouncing(CollisionTypes.WALL_CORNER)
    '''BallTests.test_kick(-1, 0)  # player on the left of the ball -> kick East
    BallTests.test_kick(0, 1)  # player above the ball -> kick South
    BallTests.test_kick(1, -1)  # player right and below the ball -> kick North-West
    BallTests.test_kick(1, -3)  # player right and below the ball -> kick North-West
    BallTests.test_receive()
    BallTests.test_collision_with_round_player(player_pos_x=0, player_pos_y=10, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=0, ball_vel_y=5,)
    BallTests.test_collision_with_round_player(player_pos_x=10, player_pos_y=0, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=5, ball_vel_y=0, )
    BallTests.test_collision_with_round_player(player_pos_x=10, player_pos_y=-10, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=5, ball_vel_y=-5, dt=0.001)
    BallTests.test_collision_with_round_player(player_pos_x=10, player_pos_y=-10.1, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=5, ball_vel_y=-5, dt=0.001)
    BallTests.test_collision_with_round_player(player_pos_x=10, player_pos_y=-9.9, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=5, ball_vel_y=-5, dt=0.001)'''
    BallTests.test_collision_with_round_player(player_pos_x=6, player_pos_y=10, player_vel_x=0, player_vel_y=0,
                                               ball_pos_x=0, ball_pos_y=0, ball_vel_x=3, ball_vel_y=5, dt=0.001)

    # TODO: add tests for the moving player
