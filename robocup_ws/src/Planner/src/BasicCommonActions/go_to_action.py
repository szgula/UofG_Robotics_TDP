import numpy as np
from game_interfaces.msg import Position

def calculate_angle_difference(alpha, beta):
    """
    Return min angle in rad in range <-pi, pi>
    """
    diff = alpha - beta
    diff = clip_angle(diff)
    return diff

def clip_angle(angle):
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    return angle


def simple_go_to_action(robot_state: Position, target: Position):
    MIN_PURE_ROTATION_ANGLE = np.pi / 6  # 30 deg
    K_P_PURE_ROTATION = 3
    MAX_OUTPUT_VALUR = 3
    K_P_FORWARD_COMPONENT = 3

    dx = target.x - robot_state.x
    dy = target.y - robot_state.y
    d = np.hypot(dx, dy)
    target_angle = np.arctan2(dy, dx)
    angle_diff = calculate_angle_difference(target_angle, robot_state.theta)

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


def go_to_fast(robot_state: Position, target: Position):
    MIN_PURE_ROTATION_ANGLE = np.pi / 36
    K_P_PURE_ROTATION = 10
    MAX_OUTPUT_VALUR = 3
    K_P_FORWARD_COMPONENT = 50

    dx = target.x - robot_state.x
    dy = target.y - robot_state.y
    d = np.hypot(dx, dy)
    target_angle = np.arctan2(dy, dx)
    angle_diff = calculate_angle_difference(target_angle, robot_state.theta)

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


def receive_and_pass_action(robot_state: Position, pass_target: Position, ball_position: Position):
    K_P_PURE_ROTATION = 3

    dx_ball_player = ball_position.x - robot_state.x
    dy_ball_player = ball_position.y - robot_state.y
    ball_player_angle = np.arctan2(dy_ball_player, dx_ball_player)

    dx_ball_target = ball_position.x - pass_target.x
    dy_ball_target = ball_position.y - pass_target.y
    ball_target_angle = np.arctan2(dy_ball_target, dx_ball_target)

    angle_diff = calculate_angle_difference(ball_target_angle, ball_player_angle)
    if abs(angle_diff) < np.deg2rad(177):
        if angle_diff > 0:
            new_heading = clip_angle(ball_player_angle + np.pi/2)
            #new_heading = clip_angle(new_heading + np.pi)
        else:
            new_heading = clip_angle(ball_player_angle - np.pi/2)
            #new_heading = clip_angle(new_heading)
        robot_heading_diff = calculate_angle_difference(robot_state.theta, new_heading)
        if abs(robot_heading_diff) > np.deg2rad(3):
            vel_l, vel_r, action, _ = rotate_towards(robot_state, new_heading)
        else:
            vel_l, vel_r = go_around_the_point(robot_state, None, 0.05)
            action = 0
    else:
        vel_l = 0
        vel_r = 0
        action = 1
    return vel_l, vel_r, action

def go_around_the_point(robot_state: Position, go_around_point: Position, radius, direction=1, R_vel = 1):
    """
    direction: 1 = clockwise, -1 = counterclockwise
    """
    l = 0.05  # distance between wheels
    K = 2 * radius / l
    C = (K-1) / (K+1)
    L_vel = R_vel * C
    #L_vel *= -1
    #R_vel *= -1
    if direction == 1:
        return L_vel, R_vel
    elif direction == -1:
        return R_vel, L_vel


def rotate_towards(robot_state: Position, target_heading):
    K_P_PURE_ROTATION = 6
    angle_diff = calculate_angle_difference(target_heading, robot_state.theta)
    done = False
    if abs(angle_diff) > np.deg2rad(3):
        vel_l = angle_diff * K_P_PURE_ROTATION
        vel_r = -angle_diff * K_P_PURE_ROTATION
        action = 0
    else:
        vel_l = 0
        vel_r = 0
        action = 2
        done = True
    return vel_l, vel_r, action, done

def stop_the_ball():
    return 0, 0, 2

