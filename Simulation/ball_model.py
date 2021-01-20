from abc import ABC, abstractmethod


class RobotModel(ABC):
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
