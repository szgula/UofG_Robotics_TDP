from abc import ABC, abstractmethod
import numpy as np
from GameEngine.ball_model import BallActions
import matplotlib.pyplot as plt
from GameEngine.collisions import CollisionTypes


class RobotModel(ABC):
    @abstractmethod
    def __init__(self, init_x_pos: float, init_y_pos: float, dt: float = 0.1, robot_radius: float = 0.1, wheel_radius: float = 0.02, cord_system_rot: float = 0, axis_len: float = 0.05):
        """
        :param dt: simulation time step
        """
        self._dt = dt                                       # simulation time step
        self.pointing_angle = 0                             # robot pointing direction, range <-pi, pi>
        self.vel = 0                                        # robot velocity
        self.vel_limit = (-float('inf'), float('inf'))      # robot velocity limits

        # Ego field coordinate system (EFCS): located in the middle of the field,
        # positive X towards opponent's goal
        # positive Y 90deg rotated counterclockwise from X axis
        # pointing angle [rad], 0rad with X axis, positive towards the Y axis (counterclockwise)
        self._cord_system_rot = cord_system_rot  # team coordinate system (EFCS) rotation in respect to game CS
        x_init, y_init = self._convert_field_CS_to_EFCS(init_x_pos, init_y_pos)
        self._x_pos_EFCS = x_init                           # robot x coordinate in field coordinate system
        self._y_pos_EFCS = y_init                           # robot x coordinate in field coordinate system
        self.radius = robot_radius                          # assuming round robot
        self._wheel_radius = wheel_radius                   # wheel_radius
        self._axis_len = axis_len                           # distance between wheels

        #self.last_collision_object = None
        #self.steps_since_last_collision = 1000

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
    def step(self, l_motor_speed: float, r_motor_speed: float, extra_action: BallActions = BallActions.NO) -> BallActions:
        """
        Execute simulation step
        :return:
        """
        pass

    @abstractmethod
    def collision_step(self, collision_object, collision_type, l_motor_speed: float, r_motor_speed: float, extra_action: BallActions = BallActions.NO):
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
        super().__init__(*args, **kwargs)

    def step(self, l_motor_speed: float, r_motor_speed: float, extra_action: BallActions = BallActions.NO) -> BallActions:
        """
        Assuming differential drive
        :param l_motor_speed: <rad/s> left motor rotation speed
        :param r_motor_speed: <rad/s> right motor rotation speed
        :param extra_action: <None, kick, receive>
        :return:
        """
        self.vel = (self._wheel_radius / 2.0) * (l_motor_speed + r_motor_speed)
        xn = self._x_pos_EFCS + (self._wheel_radius * self._dt / 2.0) * (l_motor_speed + r_motor_speed) * np.cos(self.pointing_angle)
        yn = self._y_pos_EFCS + (self._wheel_radius * self._dt / 2.0) * (l_motor_speed + r_motor_speed) * np.sin(self.pointing_angle)
        qn = self.pointing_angle + (self._wheel_radius * self._dt / (self._axis_len)) * (l_motor_speed - r_motor_speed)
        self._x_pos_EFCS, self._y_pos_EFCS, self.pointing_angle = xn, yn, qn
        return extra_action

    def collision_step(self, collision_object, collision_type: CollisionTypes, l_motor_speed: float, r_motor_speed: float, extra_action: BallActions = BallActions.NO):
        """
        Define behavior in contact with other robot/ball/wall
        :return:
        """
        if collision_type == CollisionTypes.PLAYER:
            other_pos_x, other_pos_y = collision_object.get_position_components_wcs()
            self_pos_x, self_pos_y = self.get_position_components_wcs()
            diff_x, diff_y = self_pos_x - other_pos_x, self_pos_y - other_pos_y
            collision_angle = np.arctan2(diff_y, diff_x)
            angle_diff = abs((collision_angle - self.pointing_angle + np.pi) % (2*np.pi) - np.pi)
            if angle_diff > np.pi / 2:   # the move direction need to be grater then... TODO
                self.step(l_motor_speed, r_motor_speed, extra_action)
            else:  # robot is heading towards the collision object
                if max(l_motor_speed, l_motor_speed) < 0:  # robot is going backwards - away from collision
                    self.step(l_motor_speed, r_motor_speed, extra_action)
                else:
                    speed_cancellation = max(l_motor_speed, l_motor_speed)
                    self.step(l_motor_speed-speed_cancellation, r_motor_speed-speed_cancellation, extra_action)

        # TODO
        elif collision_type != CollisionTypes.No:
            pass


class RobotModelTests:

    @staticmethod
    def t_plot(history):
        f, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(history['x'], history['y'], label='pos')
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

    @staticmethod
    def add_history(history, t, x, y, dx, dy):
        history['x'].append(x)
        history['y'].append(y)
        history['t'].append(t)
        history['dx'].append(dx)
        history['dy'].append(dy)

    @staticmethod
    def test_stationary(x=0, y=0, time=100):
        robot = RobotBasicModel(x,y)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(int(time / robot._dt)):
            robot.step(0,0)
            x, y = robot.get_position_components_wcs()
            history['x'].append(x)
            history['y'].append(y)
            history['t'].append(i)
        plt.plot(history['t'], history['x'], label='x_pos')
        plt.plot(history['t'], history['y'], label='y_pos')
        return True

    @staticmethod
    def test_moving_forward(direction=1, x=0, y=0, dq=5, time=100):
        robot = RobotBasicModel(x, y)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(int(time / robot._dt)):
            robot.step(direction * dq, direction * dq)
            x, y = robot.get_position_components_wcs()
            dx, dy = robot.get_velocity_components_wcs()
            RobotModelTests.add_history(history, i * robot._dt, x, y, dx, dy)

        RobotModelTests.t_plot(history)
        return True

    @staticmethod
    def test_moving_round(speed_l=5, speed_r=4, x=0, y=0, time=100):
        robot = RobotBasicModel(x, y)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(int(time / robot._dt)):
            robot.step(speed_l, speed_r)
            x, y = robot.get_position_components_wcs()
            dx, dy = robot.get_velocity_components_wcs()
            RobotModelTests.add_history(history, i * robot._dt, x, y, dx, dy)

        RobotModelTests.t_plot(history)
        return True

    @staticmethod
    def test_rotation(speed=5, x=0, y=0, time=5):
        robot = RobotBasicModel(x, y)
        history = {'x': [], 'y': [], 'dx': [], 'dy': [], 't': []}

        for i in range(int(time / robot._dt)):
            robot.step(speed, -speed)
            x, y = robot.get_position_components_wcs()
            dx, dy = robot.get_velocity_components_wcs()
            RobotModelTests.add_history(history, i * robot._dt, x, y, dx, dy)

        RobotModelTests.t_plot(history)
        return True

    @staticmethod
    def test_coordinate_frame_conversion():
        pass

    @staticmethod
    def test_collision():
        pass

if __name__ == "__main__":
    #RobotModelTests.test_stationary()
    #RobotModelTests.test_moving_forward(direction=-1)
    RobotModelTests.test_moving_round(speed_l=-5, speed_r=-4)
    #RobotModelTests.test_rotation()