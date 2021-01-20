from abc import ABC, abstractmethod


class RobotModel(ABC):
    @abstractmethod
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1):
        """
        :param dt: simulation time step
        """
        self._dt = dt
        self._pointing_angle = 0                            # robot pointing direction, range <-pi, pi>
        self.vel = 0                                        # robot velocity
        self.vel_limit = (-float('int'), float('int'))      # robot velocity limits

        # Ego field coordinate system (EFCS): located in the middle of the field,
        # positive X towards opponent's goal
        # positive Y 90deg rotated counterclockwise from X axis
        x_init, y_init = self._convert_field_CS_to_EFCS(init_x_pos, init_y_pos)
        self._x_pos_EFCS = x_init                           # robot x coordinate in field coordinate system
        self._y_pos_EFCS = y_init                           # robot x coordinate in field coordinate system

    @staticmethod
    @abstractmethod
    def _convert_field_CS_to_EFCS(pos_x:float, pos_y:float) -> (float, float):
        """
        Convert coordinate systems
        :param pos_x:
        :param pos_y:
        :return:
        """
        return 0, 0

    @staticmethod
    @abstractmethod
    def _convert_EFCS_to_field_CS(pos_x: float, pos_y: float) -> (float, float):
        """
        Convert coordinate systems
        :param pos_x:
        :param pos_y:
        :return:
        """
        return 0, 0

    @abstractmethod
    def show_possible_actions(self):
        """
        Provide info about robot actions (move, rotate, kick, receive the ball...)
        :return:
        """
        pass

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
