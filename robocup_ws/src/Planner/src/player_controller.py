from robot_control import Robot, Goal
from game_interfaces.msg import PlayerCommand
from brachistochrone import cycloid
import numpy as np
import matplotlib.pyplot as plt
from game_interfaces.msg import Position

class PlayerController(Robot):
    def __init__(self):
        self.points_to_visit = []
        pass

    def get_coordinates(self, my_pos_efcs, ball_pos_efcs):
        x_trajectory, y_trajectory, T = cycloid(my_pos_efcs.x + 5,my_pos_efcs.y + 3,ball_pos_efcs.x + 5,ball_pos_efcs.y + 3,2)
        print(x_trajectory,y_trajectory)
        for x,y in zip(x_trajectory,y_trajectory):
            self.points_to_visit.append(Position(x,y,0))
        return self.points_to_visit

    def get_action(self, traj_x, traj_y):

        def go_to_fast(robot_x, robot_y, target_x, target_y):

            def calculate_angle_difference(alpha, beta):
                """
                Return min angle in rad in range <-pi, pi>
                """
                diff = alpha - beta
                if diff > np.pi:
                    diff -= 2 * np.pi
                elif diff < -np.pi:
                    diff += 2 * np.pi
                return diff


            MIN_PURE_ROTATION_ANGLE = np.pi / 36
            K_P_PURE_ROTATION = 10
            MAX_OUTPUT_VALUR = 3
            K_P_FORWARD_COMPONENT = 50
            dx = target_x - robot_x
            dy = target_y - robot_y
            d = np.hypot(dx, dy)
            target_angle = np.arctan2(dy, dx)
            angle_diff = calculate_angle_difference(target_angle, np.pi/2)
            rotation_component_l = angle_diff * K_P_PURE_ROTATION
            rotation_component_r = -angle_diff * K_P_PURE_ROTATION
            if abs(angle_diff) > MIN_PURE_ROTATION_ANGLE:
                forward_component = 0
            else:
                forward_component = min(d, 1) * K_P_FORWARD_COMPONENT

            vel_l = rotation_component_l + forward_component
            vel_r = rotation_component_r + forward_component
            vel_l = np.clip(vel_l, -MAX_OUTPUT_VALUR, MAX_OUTPUT_VALUR)
            vel_r = np.clip(vel_r, -MAX_OUTPUT_VALUR, MAX_OUTPUT_VALUR)

            return vel_l, vel_r

        i = 1
        j = 1
        lw = []
        rw = []
        while(i < len(traj_x) and j < len(traj_y)):
            curr_x = traj_x[i - 1]
            curr_y = traj_y[i - 1]
            dest_x = traj_x[i]
            dest_y = traj_y[i]
            Vl , Vr = go_to_fast(curr_x, curr_y, dest_x, dest_y)
            lw.append(Vl)
            rw.append(Vr)
            i+=1
            j+=1
        print(lw)
        print(rw)
        plt.plot(traj_x,traj_y)
        plt.show()
        return lw, rw
