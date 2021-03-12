import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rospy
from robot_model import RobotModel, RobotBasicModel
from ball_model import BallModel, BallActions, BallBasicModel
from collisions import CollisionTypes
from visualizer import BasicVisualizer
from game_interfaces.srv import SimulationUpdate, SimulationUpdateResponse
from game_interfaces.msg import TeamPosition
import argparse
import random

VISUALIZER = True


class GameSimulator:
    def __init__(self, robot_class: RobotModel, ball_model: BallModel, number_of_teams: int = 2, number_of_robots: int = 5,
                 size_of_field: tuple = (10, 6), dt: float = 0.1,
                 team_0_starting_points: list = None,
                 team_1_starting_points: list = None,
                 ball_init_pos: tuple = None,
                 ball_init_vel: tuple = None):
        """

        Ego field coordinate system: located in the middle of the field, positive X towards opponent's goal
        positive Y 90deg rotated counterclockwise from X axis
        :param robot_class:
        :param ball_model:
        :param number_of_teams:
        :param number_of_robots:
        :param size_of_field:
        """
        self.dt = dt
        self._robot_class = robot_class
        self._number_of_teams = number_of_teams
        self._number_of_robots = number_of_robots
        self._size_of_field = size_of_field
        self._robots = [list() for _ in range(self._number_of_teams)]
        self.size_of_net = 2
        self.team_CS_rotations = [0, np.pi]
        self.team_0_starting_points = team_0_starting_points
        self.team_1_starting_points = team_1_starting_points

        if self.team_0_starting_points is None:
            self.team_0_starting_points = [(-3, 2), (-3, -2), (-1, 2), (-1, -2), (-4, 0)]
        if self.team_1_starting_points is None:
            self.team_1_starting_points = [(-3, 2), (-3, -2), (-1, 2), (-1, -2), (-4, 0)]

        for player_id in range(self._number_of_robots):
            self._robots[0].append(
                self._robot_class(*self.team_0_starting_points[player_id], cord_system_rot=self.team_CS_rotations[0], dt=self.dt))
            self._robots[1].append(
                self._robot_class(*self.team_1_starting_points[player_id], cord_system_rot=self.team_CS_rotations[1], dt=self.dt))
        rand_x = random.uniform(-4.6, 4.6)
        rand_y = random.uniform(-2.6,2.6)
        ball_pos = ball_init_pos if ball_init_pos is not None else (0, 0)
        ball_vel = ball_init_vel if ball_init_vel is not None else (0, 0)
        self.ball = ball_model(ball_pos[0], ball_pos[1], init_x_vel=ball_vel[0], init_y_vel=ball_vel[1], dt=self.dt)
        self._internal_goal_counter = [0, 0]

    def reset(self):
        """
        Reset simulation to initial condition
        """
        self.ball.reset()
        for team in range(self._number_of_teams):
            for robot_ in self._robots[team]:
                robot_.reset()

    def get_robot_model(self, team_id: int, player_id: int):
        return self._robots[team_id][player_id]

    def step(self, actions_per_team_per_player) -> bool:
        """
        Execute the simulation step for all components
        In this version the steps are sequential -> team 1 players, team 2 players, ball
        :param actions_per_team_per_player:
        :return: step execution status
        """
        other_players_actions = []
        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                action = actions_per_team_per_player[team].players_commands[player_id]
                action = [action.left_rpm, action.right_rpm, BallActions(action.extra_action)]
                any_collision, *collisions = self.check_for_player_collisions(team, player_id)

                if not any_collision:
                    other_action = self._robots[team][player_id].step(*action)
                    if other_action != BallActions.NO:
                        other_players_actions.append((self._robots[team][player_id], other_action))
                else:
                    self._robots[team][player_id].collision_step(*collisions, *action)

        ball_any_collision, *ball_collisions = self.check_ball_collisions()
        if ball_any_collision:
            self.ball.collision_step(*ball_collisions)
        if len(other_players_actions) != 0:
            self.ball.players_actions(other_players_actions)
        self.ball.step()
        goal_status = self.check_if_ball_in_net()
        # ball_status = self.check_if_ball_out(self._internal_goal_counter)
        return True, goal_status

    def get_positions_for_visualizer(self):
        """
        Get simulation state to update visualizer,
        also handle the conversion to the visualizer format (FIXME: should be in the visualizer not here)
        :return:
        """
        ball = self.ball.get_position()
        ball = np.array([ball[0], -ball[1]])  # FIXME: minus is due to inverted axis in pygames
        team_0, team_1 = [], []
        for player_id in range(self._number_of_robots):
            tp0_x, tp0_y = self._robots[0][player_id].get_position_components_wcs()
            tp0_angle = self._robots[0][player_id].get_pointing_angle_wcs()
            team_0.append([tp0_x, -tp0_y, tp0_angle])  # FIXME: minus is due to inverted axis in pygames
            tp1_x, tp1_y = self._robots[1][player_id].get_position_components_wcs()
            tp1_angle = self._robots[1][player_id].get_pointing_angle_wcs()
            team_1.append([tp1_x, -tp1_y, tp1_angle])  # FIXME: minus is due to inverted axis in pygames

        team_0 = np.array(team_0) + np.hstack((np.array(self._size_of_field) / 2, [0]))
        team_1 = np.array(team_1) + np.hstack((np.array(self._size_of_field) / 2, [0]))
        ball = ball + np.array(self._size_of_field) / 2
        return team_0, team_1, ball, self._internal_goal_counter

    def get_positions(self):
        """
        FIXME: should return positions of all robots in teams - currently it is fixed for single team
        FIXME: .... the above is done, but verify if any calls to this method need to be modified
        :return:
        """
        x_pos_1, x_pos_2 = [], []
        y_pos_1, y_pos_2 = [], []
        for player_id in range(self._number_of_robots):
            x, y = self._robots[0][player_id].get_position_components_wcs()
            x_pos_1.append(x), y_pos_1.append(y)
            x, y = self._robots[1][player_id].get_position_components_wcs()
            x_pos_2.append(x), y_pos_2.append(y)

        return np.array([x_pos_1, y_pos_1]), np.array([x_pos_2, y_pos_2])

    def check_ball_collisions(self):
        x, y = self.ball.get_position()
        wall_collision = self.check_wall_collisions(x, y)
        players_collision, collisions_with = self.check_players_collisions(x, y, 0, -1, -1)
        any_collision = True
        if wall_collision == CollisionTypes.NO and players_collision == CollisionTypes.NO:
            any_collision = False
        return any_collision, wall_collision, players_collision, collisions_with

    def check_for_player_collisions(self, team_id, player_id):
        """
        Check all possible collisions of the player (with walls, other players)
        Ball collision is not included - as we assume it do not change the player kinematics/dynamics
        :param team_id:
        :param player_id:
        :return:
        """
        robot = self._robots[team_id][player_id]
        wall_collision = self.check_wall_collisions(*robot.get_position_components_wcs())
        players_collision, collisions_with = self.check_players_collisions(*robot.get_position_components_wcs(),
                                                                                  robot.radius, player_id, team_id)
        any_collision = True
        if wall_collision == CollisionTypes.NO and players_collision == CollisionTypes.NO:
            any_collision = False
        return any_collision, wall_collision, players_collision, collisions_with

    def check_wall_collisions(self, x, y):
        """
        Check position is in collision with wall - if so which one
        :param player:
        :return:
        """
        vert, horiz = False, False
        if x >= self._size_of_field[0] / 2 or x <= -self._size_of_field[0] / 2:
            vert = True
        if y >= self._size_of_field[1] / 2 or y <= -self._size_of_field[1] / 2:
            horiz = True

        if horiz and vert: collision_type = CollisionTypes.WALL_CORNER
        elif horiz: collision_type = CollisionTypes.WALL_HORIZONTAL
        elif vert: collision_type = CollisionTypes.WALL_VERTICAL
        else: collision_type = CollisionTypes.NO
        return collision_type

    def check_ball_player_collision(self, player: RobotModel):
        """
        Check if player is in collision with ball
        :param player:
        :return:
        """
        p_x, p_y = player.get_position_components_wcs()
        b_x, b_y = self.ball.get_position()
        p_r, b_r = player.radius, self.ball.radius

        d = np.hypot(p_x - b_x, p_y - b_y)
        safe_collision_threshold = 1.2  # FIXME
        R = (p_r + b_r) * safe_collision_threshold
        if d <= R:
            return CollisionTypes.BALL
        else:
            return CollisionTypes.NO

    def check_players_collisions(self, p_x, p_y, R, this_player_id, this_team_id):
        """
        Check if object is in collision with other players, list all collisions
        :param p_x: object to check x coordinate
        :param p_y: object to check y coordinate
        :param R: object to check radius
        :param this_player_id: object to check team id (if not in team: -1)
        :param this_team_id: object to check player id (if not a player: -1)
        :return:
        """
        safe_collision_threshold = 1.1  # FIXME
        #p_x, p_y = player.get_position_components_wcs()
        R = R + self._robots[0][0].radius  * safe_collision_threshold
        collisions_list = []

        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                if team == this_team_id and player_id == this_player_id:
                    continue
                o_x, o_y = self._robots[team][player_id].get_position_components_wcs()
                collision = np.hypot(p_x - o_x, p_y - o_y) <= R
                if collision:
                    collisions_list.append(self._robots[team][player_id]) #(team, player_id))
        collision = CollisionTypes.PLAYER if len(collisions_list) else CollisionTypes.NO
        return collision, collisions_list

    def check_if_ball_in_net(self) -> int:
        """
        :return: 0 no goal, or id of team who score a goal
        """
        ball_pos = self.ball.get_position()
        field_size = self._size_of_field
        goal_state = 0
        ball_out_x = [-4.7, 4.7]
        ball_out_y = [-2.7, 2.7]
        ball_in_net_x_threshold = 0.4
        if field_size[0]/2 - abs(ball_pos[0]) < ball_in_net_x_threshold:
            if abs(ball_pos[1]) <= self.size_of_net/2:
                if ball_pos[0] > 0:
                    goal_state = 1
                    self._internal_goal_counter[0] += 1
                else:
                    goal_state = 2
                    self._internal_goal_counter[1] += 1
                self.reset()
        elif (ball_pos[0] < ball_out_x[0] or ball_pos[0] > ball_out_x[1] or ball_pos[1] < ball_out_y[0] or ball_pos[1] > ball_out_y[1]):
            self.reset()
        return goal_state


class GameSimulationServer(GameSimulator):
    def __init__(self, *args, **kwargs):
        self.visualizer = BasicVisualizer(None)
        super().__init__(RobotBasicModel, BallBasicModel, *args, **kwargs)
        rospy.init_node('game_simulation_server')
        s = rospy.Service(r'game_engine/game_simulation', SimulationUpdate, self.handle_simulation_call)
        print("Ready to simulate the game.")
        rospy.spin()

    def handle_simulation_call(self, simulation_request):
        update_status, goal_status = False, False
        if not simulation_request.reset and simulation_request.update:
            update_status, goal_status = self.step(simulation_request.teams_commands)
        response = self._generate_response_message(update_status, goal_status)
        if VISUALIZER:
            self.visualizer.send_game_state(*self.get_positions_for_visualizer())
            self.visualizer.display()

        return response

    def _generate_response_message(self, update_status=False, goal_status=False) -> SimulationUpdateResponse:
        team_pos = [TeamPosition(), TeamPosition()]

        ball_pos_wcs, ball_pos_efcs1 = self.ball.get_position_for_ros_srv()
        ball_vel_wcs, ball_vel_efcs1 = self.ball.get_velocity_for_ros_srv()
        team_pos[0].ball_pos_efcs, team_pos[1].ball_pos_efcs = ball_pos_wcs, ball_pos_efcs1
        team_pos[0].ball_vel_efcs, team_pos[1].ball_vel_efcs = ball_vel_wcs, ball_vel_efcs1

        for team_idx in range(self._number_of_teams):
            team_pos[team_idx].team_id = team_idx
            for player_idx in range(self._number_of_robots):
                team_pos_wcs, team_pos_efcs0 = self._robots[team_idx][player_idx].get_position_for_ros_srv()
                team_pos[team_idx].players_positions_wcs[player_idx] = team_pos_wcs
                team_pos[team_idx].players_positions_efcs[player_idx] = team_pos_efcs0

        return SimulationUpdateResponse(update_status, goal_status, team_pos, ball_pos_wcs)


if __name__ == "__main__":
    my_parser = argparse.ArgumentParser(description='Simulation config')
    my_parser.add_argument('--ball_pos',
                           type=float,
                           nargs='+',
                           help='ball initial pos in wcs: 2 floats (x, y)',
                           required=False)
    my_parser.add_argument('--ball_vel',
                           type=float,
                           nargs='+',
                           help='ball initial vel in wcs: 2 floats (x, y)',
                           required=False)
    my_parser.add_argument('--team_0_init_pos',
                           type=float,
                           nargs='+',
                           help='team 0 initial pos in wcs: 10 floats (x0, y0, x1, y1...)',
                           required=False)
    my_parser.add_argument('--team_1_init_pos',
                           type=float,
                           nargs='+',
                           help='team 0 initial pos in wcs: 10 floats (x0, y0, x1, y1...)',
                           required=False)

    # Remove some roslaunch artefacts
    idx_to_rem = []
    for arg_idx, arg_name in enumerate(sys.argv):
        if "__name:=game_simulator" in arg_name or "__log:=" in arg_name:
            idx_to_rem.append(arg_idx)
    for idx in idx_to_rem[::-1]:
        sys.argv.pop(idx)

    # Parse arguments
    args_cl, unknown = my_parser.parse_known_args()
    team_0_starting_points = args_cl.team_0_init_pos
    team_1_starting_points = args_cl.team_1_init_pos
    ball_init_pos = args_cl.ball_pos
    ball_init_vel = args_cl.ball_vel

    # reformat positions
    if team_0_starting_points is not None:
        team_0_starting_points = [(team_0_starting_points[2 * i], team_0_starting_points[2 * i + 1]) for i in range(5)]
    if team_1_starting_points is not None:
        team_1_starting_points = [(team_1_starting_points[2 * i], team_1_starting_points[2 * i + 1]) for i in range(5)]
    GSS = GameSimulationServer(team_0_starting_points=team_0_starting_points,
                               team_1_starting_points=team_1_starting_points,
                               ball_init_pos=ball_init_pos,
                               ball_init_vel=ball_init_vel)
