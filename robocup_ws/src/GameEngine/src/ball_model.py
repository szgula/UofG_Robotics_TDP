from abc import ABC, abstractmethod
from enum import Enum
from src.collisions import CollisionTypes
import logging
import numpy as np

class BallModel(ABC):
    @abstractmethod
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1):
        """
        :param dt: simulation time step
        """
        self._dt = dt
        self._x_vel = 0                                     # robot velocity
        self._y_vel = 0                                     # robot velocity
        self.vel_limit = (-float('int'), float('int'))      # robot velocity limits
        self._x_pos = init_x_pos                            # ball x coordinate in field coordinate system
        self._y_pos = init_y_pos                            # ball y coordinate in field coordinate system

        self.last_collision_object = None
        self.steps_since_last_collision = 1000

    @abstractmethod
    def step(self, action):
        """
        Execute simulation step
        :return:
        """
        pass

    @abstractmethod
    def reset(self):
        """
        Reset ball state to initial condition
        :return:
        """
        pass

    @abstractmethod
    def collision_step(self,  wall_collision:CollisionTypes, players_collision:CollisionTypes, collisions_with:list):
        """
        Define behavior in contact with other robot/ball/wall
        :return:
        """
        pass


class BallActions(Enum):
    NO = 0
    KICK = 1
    RECEIVE = 2


class BallBasicModel(BallModel):
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1, friction:float = 0.01, mass:float = 0.1,
                 vel_bounce_coef: float = 0.8, radius:float = 0.01, ball_max_vel: float = 3):
        """
        :param dt: simulation time step
        """
        self._dt = dt
        self._x_vel = 0                                     # robot velocity x component
        self._y_vel = 0                                     # robot velocity y component
        self._max_vel = ball_max_vel                        # robot velocity limits
        self._init_pos = [init_x_pos, init_y_pos]
        self._x_pos = init_x_pos                            # ball x coordinate in field coordinate system
        self._y_pos = init_y_pos                            # ball y coordinate in field coordinate system
        self._friction = friction                           # friction proportional to vel
        self._mass = mass                                   # ball mass
        self._vel_bounce_coefficient = vel_bounce_coef      # coefficient of velocity that is preserved after bounce
        self.radius = radius                                # ball dimensions

        self.last_collision_object = None
        self.steps_since_last_collision = 1000
        self.MINIMAL_TIME_BETWEEN_COLLISION_WITH_SAME_OBJECT = 0

    def reset(self):
        self._x_vel, self._y_vel = 0, 0
        self._x_pos, self._y_pos = self._init_pos[0], self._init_pos[1]
        self.last_collision_object = None
        self.steps_since_last_collision = 1000

    def step(self):
        """
        Execute simulation step
        :return:
        """
        vel_sq = (self._x_vel**2 + self._y_vel**2)
        vel = vel_sq**0.5
        acc = -vel * self._friction / self._mass

        acceleration_threshold = 0.01
        if abs(acc) > acceleration_threshold:
            acc_x = acc * self._x_vel / vel
            acc_y = acc * self._y_vel / vel
        else:
            acc_x, acc_y = 0, 0
        dt_sq = self._dt ** 2
        self._x_pos += self._x_vel * self._dt + 0.5 * acc_x * dt_sq
        self._y_pos += self._y_vel * self._dt + 0.5 * acc_y * dt_sq
        self._x_vel += acc_x * self._dt
        self._y_vel += acc_y * self._dt
        # TODO: how about collision step?
        self.steps_since_last_collision += 1

    def players_actions(self, actions):
        if len(actions) > 1:
            logging.warning("Thy multiple actions are not supported yet - executing first action")
        object = actions[0][0]
        action_type = actions[0][1]
        import pdb; pdb.set_trace()
        if action_type == BallActions.KICK:
            self._kick_collision(object)
        elif action_type == BallActions.RECEIVE:
            self._receive_collision(object)

    def collision_step(self,  wall_collision:CollisionTypes, players_collision:CollisionTypes, collisions_with:list):
        """
        Define behavior in contact with other robot/ball/wall
        :param collision_object:
        :param collision_type: can be "moving", "wall", "kick", "receive"
        :return:
        """
        if (wall_collision != CollisionTypes.NO and players_collision != CollisionTypes.NO) or len(collisions_with) > 1:
            logging.warning("Thy multiple collision are not supported yet")
            if wall_collision != CollisionTypes.NO:
                logging.warning("Handling just wall collision")
                players_collision = CollisionTypes.NO
            else:
                logging.warning("Handling just first player collision")
                collisions_with = [collisions_with[0]]
            self._wall_collision(wall_collision)

        if wall_collision != CollisionTypes.NO:
            self._wall_collision(wall_collision)
        if players_collision != CollisionTypes.NO:
            if self.steps_since_last_collision <= self.MINIMAL_TIME_BETWEEN_COLLISION_WITH_SAME_OBJECT and \
                    collisions_with == self.last_collision_object:
                pass
            else:
                collisions_with = collisions_with[0]  # TODO: handles just one instance for now
                self._elastic_collision_with_round_player(collisions_with)
                self.steps_since_last_collision = 0
                self.last_collision_object = collisions_with
        #self.step()

    def _wall_collision(self, collision_type:CollisionTypes):
        """
        Ball bounce from the static wall - can be horizontal or vertical
        :param collision_object:
        :param wall_orientation: "vertical", "horizontal", "corner"
        :param kwargs:
        :return:
        """
        if collision_type == CollisionTypes.WALL_VERTICAL or collision_type == CollisionTypes.WALL_CORNER:
            self._x_vel *= -1 * self._vel_bounce_coefficient
            self._x_pos = np.round(self._x_pos)  # TODO assuming the wall is on "rounded" position - may not be the case
        if collision_type == CollisionTypes.WALL_HORIZONTAL or collision_type == CollisionTypes.WALL_CORNER:
            self._y_vel *= -1 * self._vel_bounce_coefficient
            self._y_pos = np.round(self._y_pos)

    def _elastic_collision_with_round_player(self, collision_object):
        """
        Ball bounce from the round player, the ball diameter is assumed 0, the ball rotation effect is mitigated
        :param collision_object:
        :return:
        """
        player_pos_x, player_pos_y = collision_object.get_position_components_wcs()
        player_radius = collision_object.radius
        dx, dy = self._x_pos - player_pos_x, self._y_pos - player_pos_y
        diff = (dx ** 2 + dy ** 2) ** 0.5  # TODO: add check if in collision range (of this time step)
        player_vx, player_vy = collision_object.get_velocity_components_wcs()

        def similar_speeds(player_vx_, player_vy_):
            speed_diff = np.hypot(player_vx_-self._x_vel, player_vy_-self._y_vel)
            same_speed_threshold = 0.01
            return speed_diff <= same_speed_threshold

        if similar_speeds(player_vx, player_vy): return

        """ Calculate intersection point between player and ball - assuming the ball radius ~= 0 for simplicity 
         Using line equation: 
         x = x0 + v_x * t 
         y = y0 + v_y * t

         Circle equation:
         (x - x0)**2 + (y-y0)**2 = r**2 """

        move_vel_threshold = 0.001              # TODO: find a better place for this
        if abs(self._x_vel) >= move_vel_threshold:
            # solving ax^2 + bx + c = 0
            k = (self._y_vel / self._x_vel)
            M = k * self._x_pos - self._y_pos + player_pos_y
            a = (1 + k ** 2)
            b = -2*(player_pos_x + M * k)
            c = player_pos_x**2 + M**2 - player_radius**2
        elif abs(self._y_vel) >= move_vel_threshold:
            a = 1
            b = -2 * player_pos_y
            c = player_pos_y**2 + (self._x_pos - player_pos_x)**2 - player_radius**2
        elif abs(player_vx) > move_vel_threshold:
            k = (player_vy / player_vx)
            M = k * player_pos_x - player_pos_y + self._y_pos  #k * self._x_pos - self._y_pos + player_pos_y
            a = (1 + k ** 2)
            b = -2 * (self._x_pos + M * k)
            c = self._x_pos ** 2 + M ** 2 - player_radius ** 2
        elif abs(player_vy) > move_vel_threshold:
            raise NotImplementedError
        else:
            raise ValueError("No collision should happen if all vel are ~0")

        #  solve quadratic equation
        delta = b ** 2 - 4 * a * c
        if delta < 0:
            return
        delta_root = delta ** 0.5
        sol1, sol2 = (-b + delta_root) / (2 * a), (-b - delta_root) / (2 * a)
        if abs(self._x_vel) >= move_vel_threshold:
            x1, x2 = sol1, sol2
            get_y = lambda _x_: self._y_pos + self._y_vel * (_x_ - self._x_pos) / self._x_vel
            y1, y2 = get_y(x1), get_y(x2)
        elif abs(self._y_vel) >= move_vel_threshold:
            x1, x2 = self._x_pos, self._x_pos
            y1, y2 = sol1, sol2
        elif abs(player_vx) > move_vel_threshold:
            x1, x2 = sol1, sol2
            get_y = lambda _x_: self._y_pos + self._y_vel * (_x_ - self._x_pos) / player_vx
            y1, y2 = get_y(x1), get_y(x2)
        elif abs(player_vy) > move_vel_threshold:
            raise NotImplementedError
        else:
            raise ValueError("No collision should happen if all vel are ~0")

        dist_1 = (x1 - self._x_pos)**2 + (y1 - self._y_pos)**2
        dist_2 = (x2 - self._x_pos)**2 + (y2 - self._y_pos)**2
        collision_x, collision_y = (x1, y1) if dist_1 < dist_2 else (x2, y2)  #  get player-ball collision point

        # https://math.stackexchange.com/questions/2239169/reflecting-a-vector-over-another-line
        incoming_vector = (self._x_vel, self._y_vel)
        m_x, m_y = incoming_vector
        normal_reflection_vector = (collision_x - player_pos_x, collision_y - player_pos_y)
        n_x, n_y = normal_reflection_vector

        # scalar product of (incoming_vector * normal_reflection_vector) / (normal_reflection_vector * normal_reflection_vector)
        sp = (m_x * n_x + m_y * n_y) / (n_x * n_x + n_y * n_y)
        r_x = m_x - 2 * sp * n_x
        r_y = m_y - 2 * sp * n_y

        vel_ = (self._x_vel**2 + self._y_vel**2)**0.5
        ref_vel = (r_x**2 + r_y**2)**0.5
        # TODO: vel_ *= self._vel_bounce_coefficient
        if abs(ref_vel) > move_vel_threshold:
            vel_x = player_vx + r_x * vel_ / ref_vel
            vel_y = player_vy + r_y * vel_ / ref_vel
        else:
            vel_x = player_vx
            vel_y = player_vy
        self._x_vel = vel_x
        self._y_vel = vel_y

    def _kick_collision(self, collision_object, kick_vel: float = 0.4, **kwargs):
        """
        The ball is kick along line connecting player's and ball's centre points.
        :param collision_object:
        :param kick_vel: absolute velocity of kicked ball
        :param kwargs:
        :return:
        """
        # TODO: check if in 'kick' range
        # TODO: take into consideration the ball incoming speed
        player_pos_x, player_pos_y = collision_object.get_position_components_wcs()
        dx, dy = self._x_pos - player_pos_x, self._y_pos - player_pos_y
        diff = (dx**2 + dy**2)**0.5
        kick_vel = min(kick_vel, self._max_vel)
        self._x_vel = kick_vel * dx / diff
        self._y_vel = kick_vel * dy / diff

    def _receive_collision(self, collision_object):
        """
        Match the ball velocity to player's velocity
        :param collision_object:
        :return:
        """
        # TODO: check if in range to receive
        self._x_vel, self._y_vel = collision_object.get_velocity_components_wcs()

    def get_position(self):
        return self._x_pos, self._y_pos


# TODO: move to separate test file and integrate it with pytest or other testing framework
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

