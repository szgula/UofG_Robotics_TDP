import numpy as np
from game_interfaces.msg import Position

def calculate_angle_difference(alpha, beta):
    """
    Return min angle in rad in range <-pi, pi>
    """
    diff = alpha - beta
    if diff > np.pi:
        diff -= 2*np.pi
    elif diff < -np.pi:
        diff += 2*np.pi
    return diff


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


