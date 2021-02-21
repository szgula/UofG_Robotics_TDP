#FIXME: imports etc, migrated form robot_model.py

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