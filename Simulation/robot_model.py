from abc import ABC, abstractmethod
import numpy as np
from .ball_model import BallActions


class RobotModel(ABC):
    @abstractmethod
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1, robot_radius: float = 0.1, wheel_radius: float = 0.02, cord_system_rot: float = 0, axis_len: float = 0.05):
        """
        :param dt: simulation time step
        """
        self._dt = dt                                       # simulation time step
        self.pointing_angle = 0                            # robot pointing direction, range <-pi, pi>
        self.vel = 0                                        # robot velocity
        self.vel_limit = (-float('int'), float('int'))      # robot velocity limits

        # Ego field coordinate system (EFCS): located in the middle of the field,
        # positive X towards opponent's goal
        # positive Y 90deg rotated counterclockwise from X axis
        # pointing angle [rad], 0rad with X axis, positive towards the Y axis (counterclockwise)
        x_init, y_init = self._convert_field_CS_to_EFCS(init_x_pos, init_y_pos)
        self._x_pos_EFCS = x_init                           # robot x coordinate in field coordinate system
        self._y_pos_EFCS = y_init                           # robot x coordinate in field coordinate system
        self.radius = robot_radius                          # assuming round robot
        self._wheel_radius = wheel_radius                   # wheel_radius
        self._axis_len = axis_len                           # distance between wheels
        self._cord_system_rot = cord_system_rot             # team coordinate system (EFCS) rotation in respect to game CS

    def _convert_field_CS_to_EFCS(self, pos_x: float, pos_y: float) -> (float, float):
        """
        Convert main game coordinate systems to team's coordinate system (EFCS)
        :param pos_x:
        :param pos_y:
        :return:
        """
        if self._cord_system_rot == 0:
            return pos_x, pos_y
        elif self._cord_system_rot == np.pi:
            return -pos_x, -pos_y
        else:
            raise ValueError("The coordinate system rotation not defined correctly")

    def _convert_EFCS_to_field_CS(self, pos_x: float, pos_y: float) -> (float, float):
        """
        Convert coordinate systems
        :param pos_x:
        :param pos_y:
        :return:
        """
        if self._cord_system_rot == 0:
            return pos_x, pos_y
        elif self._cord_system_rot == np.pi:
            return -pos_x, -pos_y
        else:
            raise ValueError("The coordinate system rotation not defined correctly")

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

    def get_velocity_components_wcs(self) -> (float, float):
        """
        Return the velocity vector (x, y components) in game coordinate system
        :return: x velocity, y velocity
        """
        return self._convert_EFCS_to_field_CS(self.vel * np.cos(self.pointing_angle),
                                              self.vel * np.sin(self.pointing_angle))

    def get_position_components_wcs(self) -> (float, float):
        """
        Return the velocity vector (x, y components) in game coordinate system
        :return: x velocity, y velocity
        """
        return self._convert_EFCS_to_field_CS(self._x_pos_EFCS, self._y_pos_EFCS)


class RobotBasicModel(RobotModel):

    def __init__(self, *args, **kwargs):
        super().__init__(args, kwargs)

    def step(self, l_motor_speed: float, r_motor_speed: float, extra_action: BallActions = BallActions.NO) -> BallActions:
        """
        Assuming differential drive
        :param l_motor_speed: <rad/s> left motor rotation speed
        :param r_motor_speed: <rad/s> right motor rotation speed
        :param extra_action: <None, kick, receive>
        :return:
        """
        xn = self._x_pos_EFCS + (self._wheel_radius * self._dt / 2.0) * (l_motor_speed + r_motor_speed) * np.cos(self._pointing_angle)
        yn = self._y_pos_EFCS + (self._wheel_radius * self._dt / 2.0) * (l_motor_speed + r_motor_speed) * np.sin(self._pointing_angle)
        qn = self._pointing_angle + (self._wheel_radius * self._dt / (self._axis_len)) * (l_motor_speed - r_motor_speed)
        self._x_pos_EFCS, self._y_pos_EFCS, self._pointing_angle = xn, yn, qn
        return extra_action

    def _collision_step(self, collision_object):
        """
        Define behavior in contact with other robot/ball/wall
        :return:
        """
        m1, m2 = self.radius ** 2, collision_object.radius ** 2
        M = m1 + m2
        r1, r2 = self.radius, collision_object.radius
        d = np.linalg.norm(r1 - r2) ** 2
        v1 = self.vel * np.array([np.cos(self._pointing_angle), np.sin(self._pointing_angle)])
        v2 = collision_object.vel * np.array([np.cos(collision_object.pointing_angle), np.sin(collision_object.pointing_angle)])
        u1 = v1 - 2 * m2 / M * np.dot(v1 - v2, r1 - r2) / d * (r1 - r2)
        self.vel = np.hypot(u1)
        self._pointing_angle = np.arctan2(u1[1], u1[0])