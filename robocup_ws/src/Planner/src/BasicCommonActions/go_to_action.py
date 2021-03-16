import numpy as np
from game_interfaces.msg import Position
from .plan_supporting_functions import TeamMasterSupporting

MAX_RPM = 3

def calculate_angle_difference(alpha, beta):
    """
    Return min angle in rad between two give angles in range <-pi, pi>
    """
    diff = alpha - beta
    diff = clip_angle(diff)
    return diff

def clip_angle(angle):
    """
    Ensure the agnel is within <-pi, pi> range
    """
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2*np.pi
    return angle

def go_to_parametrized(robot_state: Position, target: Position, MIN_PURE_ROTATION_ANGLE, K_P_PURE_ROTATION, MAX_OUTPUT_VALUR, K_P_FORWARD_COMPONENT):
    """
    Simple proportional controller
    """
    dx = target.x - robot_state.x
    dy = target.y - robot_state.y
    d = np.hypot(dx, dy)
    if d < 0.01:
        return 0, 0
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

def simple_go_to_action(robot_state: Position, target: Position):
    MIN_PURE_ROTATION_ANGLE = np.pi / 6  # 30 deg
    K_P_PURE_ROTATION = 3
    MAX_OUTPUT_VALUR = 3
    K_P_FORWARD_COMPONENT = 3
    return go_to_parametrized(robot_state, target, MIN_PURE_ROTATION_ANGLE, K_P_PURE_ROTATION, MAX_OUTPUT_VALUR, K_P_FORWARD_COMPONENT)

def go_to_fast(robot_state: Position, target: Position):
    MIN_PURE_ROTATION_ANGLE = np.pi / 36
    K_P_PURE_ROTATION = 10
    MAX_OUTPUT_VALUR = 3
    K_P_FORWARD_COMPONENT = 50
    return go_to_parametrized(robot_state, target, MIN_PURE_ROTATION_ANGLE, K_P_PURE_ROTATION, MAX_OUTPUT_VALUR, K_P_FORWARD_COMPONENT)

def boost(robot_state: Position, target: Position):
    MIN_PURE_ROTATION_ANGLE = np.pi / 36
    K_P_PURE_ROTATION = 10
    MAX_OUTPUT_VALUR = 10
    K_P_FORWARD_COMPONENT = 50
    return go_to_parametrized(robot_state, target, MIN_PURE_ROTATION_ANGLE, K_P_PURE_ROTATION, MAX_OUTPUT_VALUR, K_P_FORWARD_COMPONENT)


def move_player_inline_with_ball_and_target(robot_state: Position, ball_position: Position,
                                            ball_player_angle, ball_target_angle, inplace=True, radius=None):
    """
    This function moves the player to line crated by ball and target point
    This is used for changing robot direction while it handles the ball
    another use case is to get robot to position that allows kicking the ball towards the target

    ball_player_angle [rad] - angle between ball and player (in efcs)
    ball_target_angle [rad] - angle between ball and target (in efcs)
    inplace [bool] - if True: rotation happens inplace (ball is stopped and robot moves around it)
                     else: robot is moving forward with ball
    radius [float] - radius of the circle the robot rotation the ball with
                     if inplace == True the radius == 0
    """
    angle_diff = calculate_angle_difference(ball_target_angle, ball_player_angle)
    # First find the angle which is perpendicular to the target_ball line
    if angle_diff > 0:
        # in this case the shorter path to go around the boll is clockwise
        new_heading = clip_angle(ball_player_angle + np.pi / 2)
        heading_correction_for_change_direction = -np.deg2rad(45)
        direction = 1
    else:
        # in this case the shorter path to go around the boll is counterclockwise
        new_heading = clip_angle(ball_player_angle - np.pi / 2)
        heading_correction_for_change_direction = +np.deg2rad(45)
        direction = -1

    if not inplace:
        # if the rotation is not inplace the ball needs to be situated on the side of the robot
        # so then the robot moves, the ball is moved forward and inside the trajectory circle
        new_heading = clip_angle(new_heading + heading_correction_for_change_direction)

    robot_heading_diff = calculate_angle_difference(robot_state.theta, new_heading)
    if abs(robot_heading_diff) > np.deg2rad(3):
        # correct the robot heading
        add_forward_component = not inplace
        vel_l, vel_r, action, _ = rotate_towards(robot_state, new_heading, add_forward_component)
        if add_forward_component:
            forward_component = MAX_RPM - max(vel_l, vel_r)
            vel_l += forward_component
            vel_r += forward_component
    else:
        if inplace:
            dx_ball_player = ball_position.x - robot_state.x
            dy_ball_player = ball_position.y - robot_state.y
            r = np.hypot(dx_ball_player, dy_ball_player)
        else:
            r = radius
        vel_l, vel_r = go_around_the_point(robot_state, None, r, direction)
        action = 0
    return vel_l, vel_r, action


def get_player_ball_target_characteristic_angles(robot_state: Position, target: Position, ball_position: Position):
    """
    Returns angles between ball-player and ball_target
    """
    dx_ball_player = ball_position.x - robot_state.x
    dy_ball_player = ball_position.y - robot_state.y
    ball_player_angle = np.arctan2(dy_ball_player, dx_ball_player)

    dx_ball_target = ball_position.x - target.x
    dy_ball_target = ball_position.y - target.y
    ball_target_angle = np.arctan2(dy_ball_target, dx_ball_target)
    return ball_player_angle, ball_target_angle


def receive_and_pass_action(robot_state: Position, pass_target: Position, ball_position: Position, ball_velocity: Position):
    """
    This function generate the action to receive the ball, stop it, move ther
    """
    if np.hypot(ball_velocity.x, ball_velocity.y) > TeamMasterSupporting.max_robot_speed:
        return 0, 0, 2  # Stop the ball
    ball_player_angle, ball_target_angle = get_player_ball_target_characteristic_angles(robot_state, pass_target, ball_position)

    angle_diff = calculate_angle_difference(ball_target_angle, ball_player_angle)
    if abs(angle_diff) < np.deg2rad(177):
        vel_l, vel_r, action = move_player_inline_with_ball_and_target(robot_state, ball_position, ball_player_angle,
                                                                       ball_target_angle, inplace=True, radius=1)

    else:
        vel_l = 0
        vel_r = 0
        action = 1
    return vel_l, vel_r, action

def receive_and_dribble_action(robot_state: Position, dribble_target: Position, ball_position: Position, ball_velocity: Position):
    """
    This function generate the action to receive the ball, stop it, move to dribble target
    """
    if np.hypot(ball_velocity.x, ball_velocity.y) > TeamMasterSupporting.max_robot_speed:
        return 0, 0, 2  # Stop the ball
    ball_player_angle, ball_target_angle = get_player_ball_target_characteristic_angles(robot_state, dribble_target,
                                                                                        ball_position)

    angle_diff = calculate_angle_difference(ball_target_angle, ball_player_angle)
    if abs(angle_diff) < np.deg2rad(177):
        vel_l, vel_r, action = move_player_inline_with_ball_and_target(robot_state, ball_position, ball_player_angle,
                                                                       ball_target_angle, inplace=True, radius=1)
    else:
        vel_l, vel_r = simple_go_to_action(robot_state, dribble_target)
        action = 2
    return vel_l, vel_r, action

def go_around_the_point(robot_state: Position, go_around_point: Position, radius, direction=1, R_vel = 3):
    """
    Generate wheel velocities for robot to go around the point

    Limitation: it assumes for now the robot is already on the circle around the point

    R_vel - velocity of the faster wheel
    direction: 1 = clockwise, -1 = counterclockwise
    """
    l = 0.05  # distance between wheels
    K = 2 * radius / l
    C = (K-1) / (K+1)
    L_vel = R_vel * C
    if abs(L_vel) > MAX_RPM:  # 3 is current max rotation speed
        L_vel = R_vel
        R_vel = L_vel / C
    if direction == 1:
        return L_vel, R_vel
    elif direction == -1:
        return R_vel, L_vel

def rotate_towards(robot_state: Position, target_heading: float, with_forward_component: bool = False):
    """
    Simple controller to rotate a robot towards given heading
    with_forward_component - if true the robot rotates around the inner wheel
    """
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
    vel_l = np.clip(vel_l, -MAX_RPM, MAX_RPM)
    vel_r = np.clip(vel_r, -MAX_RPM, MAX_RPM)
    if with_forward_component:
        vel_l, vel_r = np.max(vel_l, 0), np.max(vel_r, 0)
    return vel_l, vel_r, action, done

def stop_the_ball():
    """
    Stop the ball action
    """
    return 0, 0, 2

