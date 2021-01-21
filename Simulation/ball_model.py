from abc import ABC, abstractmethod


class BallModel(ABC):
    @abstractmethod
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1):
        """
        :param dt: simulation time step
        """
        self._dt = dt
        self._heading_angle = 0                             # robot heading direction, range <-pi, pi>
        self.x_vel = 0                                       # robot velocity
        self.y_vel = 0                                      # robot velocity
        self.vel_limit = (-float('int'), float('int'))      # robot velocity limits
        self._x_pos = init_x_pos                            # ball x coordinate in field coordinate system
        self._y_pos = init_y_pos                            # ball y coordinate in field coordinate system

    @abstractmethod
    def step(self, action):
        """
        Execute simulation step
        :return:
        """
        pass

    @abstractmethod
    def _collision_step(self, collision_object):
        """
        Define behavior in contact with other robot/ball/wall
        :return:
        """
        pass


class BallBasicModel(BallModel):
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1, friction:float = 0.01, mass:float = 0.1,
                 vel_bounce_coef: float = 0.8, radius:float = 0.01, ball_max_vel: float = 3):
        """
        :param dt: simulation time step
        """
        self._dt = dt
        self._heading_angle = 0                             # robot heading direction, range <-pi, pi>
        self._x_vel = 0                                     # robot velocity x component
        self._y_vel = 0                                     # robot velocity y component
        self._max_vel = ball_max_vel                        # robot velocity limits
        self._x_pos = init_x_pos                            # ball x coordinate in field coordinate system
        self._y_pos = init_y_pos                            # ball y coordinate in field coordinate system
        self._friction = friction                           # friction proportional to vel
        self._mass = mass                                   # ball mass
        self._vel_bounce_coefficient = vel_bounce_coef      # coefficient of velocity that is preserved after bounce
        self._radius = radius                               # ball dimensions

    def step(self, action):
        """
        Execute simulation step
        :return:
        """
        vel_sq = (self._x_vel**2 + self._y_vel**2)
        vel = vel_sq**0.5
        acc = -vel * self._friction / self._mass

        acc_x = acc * abs(self._x_vel / vel)
        acc_y = acc * abs(self._y_vel / vel)
        dt_sq = self._dt ** 2
        self._x_pos += self._x_vel * self._dt + 0.5 * acc_x * dt_sq
        self._y_pos += self._y_vel * self._dt + 0.5 * acc_y * dt_sq
        self._x_vel += acc_x * self._dt
        self._y_vel += acc_y * self._dt

    def _collision_step(self, collision_object, collision_type=None, **kwargs):
        """
        Define behavior in contact with other robot/ball/wall
        :param collision_object:
        :param collision_type: can be "moving", "wall", "kick", "receive"
        :return:
        """
        pass

    def _wall_collision(self, collision_object, wall_orientation: str, **kwargs):
        """
        :param collision_object:
        :param wall_orientation: "vertical", "horizontal", "corner"
        :param kwargs:
        :return:
        """
        if wall_orientation == "vertical" or wall_orientation == "corner":
            self._y_vel *= -1 * self._vel_bounce_coefficient
        if wall_orientation == "horizontal" or wall_orientation == "corner":
            self._x_vel *= -1 * self._vel_bounce_coefficient

    def _elastic_collision_with_round_player(self, collision_object):
        """ NOT VERIFIED """
        player_pos_x, player_pos_y = collision_object.get_position_components_wcs()
        player_radius = collision_object.radius
        dx, dy = self._x_pos - player_pos_x, self._y_pos - player_pos_y
        diff = (dx ** 2 + dy ** 2) ** 0.5
        vx, vy = collision_object.get_velocity_components_wcs()

        """ Calculate intersection point between player and ball - assuming the ball radius ~= 0 for simplicity 
         Using line equation: 
         x = x0 + v_x * t  
         y = y0 + v_y * t 

         Circle equation:
         (x - x0)**2 + (y-y0)**2 = r**2 """

        move_vel_threshold = 0.001              # TODO: find a better place for this
        if self._x_vel >= move_vel_threshold:
            # solving ax^2 + bx + c = 0
            a = (1 + self._x_vel**2 / self._y_vel**2)
            M = (self._y_vel / self._x_vel) * self._x_pos - self._y_pos + player_pos_y
            b = -2*(player_pos_x - M * (self._y_vel / self._x_vel))
            c = player_pos_x**2 + M**2 - player_radius**2
        else:
            a = 1
            b = 2 * player_pos_y
            c = player_pos_y**2 + (self._x_pos - player_pos_y)**2 - player_radius**2

        #  solve quadratic equation
        delta = b ** 2 - 4 * a * c
        if delta < 0:
            return
        delta_root = delta ** 0.5
        x1, x2 = (-b + delta_root) / (2 * a), (-b - delta_root) / (2 * a)
        get_y = lambda _x_: self._y_pos + self._y_vel * (_x_ - self._x_pos) / self._x_vel
        y1, y2 = get_y(x1), get_y(x2)

        dist_1 = (x1 - self._x_pos)**2 + (y1 - self._y_pos)**2
        dist_2 = (x2 - self._x_pos)**2 + (y2 - self._y_pos)**2
        #  get player-ball collision point
        collision_x, collision_y = x1, y1 if dist_1 < dist_2 else x2, y2

        # https://math.stackexchange.com/questions/2239169/reflecting-a-vector-over-another-line
        incoming_vector = (self._x_vel, self._y_vel)
        perpendicular_mirror_vector = (collision_x - player_pos_x, collision_y - player_pos_y)
        m_x, m_y = perpendicular_mirror_vector
        mirror_vector = (1, -m_x / m_y)
        n_x, n_y = mirror_vector

        r_x = m_x - 2 * (m_x * n_x + m_y * n_y) / (n_x * n_x + n_y * n_y) * n_x
        r_y = m_y - 2 * (m_x * n_x + m_y * n_y) / (n_x * n_x + n_y * n_y) * n_y

        vel_ = (self._x_vel**2 + self._y_vel**2)**0.5
        ref_vel = (r_x**2 + r_y**2)**0.5

        # TODO: vel_ *= self._vel_bounce_coefficient

        vel_x = r_x * vel_ / ref_vel
        vel_y = r_y * vel_ / ref_vel




    def _kick_collision(self, collision_object, kick_vel: float = None, **kwargs):
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
        Match the ball velocity to players velocity
        :param collision_object:
        :return:
        """
        self._x_vel, self._y_vel = collision_object.get_velocity_components_wcs()


class BallTests:
    def test_ball_stationary(self):
        pass

    def test_ball_moving_with_initial_speed(self):
        pass

    def test_ball_vertical_wall_bouncing(self):
        pass

    def test_ball_lateral_wall_bouncing(self):
        pass

    def test_ball_corner_wall_bouncing(self):
        pass

    def test_kick(self):
        pass

    def test_receive(self):
        pass


