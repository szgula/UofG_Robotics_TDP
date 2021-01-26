import numpy as np
from GameEngine.robot_model import RobotModel
from GameEngine.ball_model import BallModel, BallActions
from GameEngine.collisions import CollisionTypes


class GameSimulator:

    def __init__(self, robot_class: RobotModel, ball_model: BallModel, number_of_teams: int = 1, number_of_robots: int = 1,
                 size_of_field: tuple = (18, 12)):
        """


        Ego field coordinate system: located in the middle of the field, positive X towards opponent's goal
        positive Y 90deg rotated counterclockwise from X axis
        :param robot_class:
        :param ball_model:
        :param number_of_teams:
        :param number_of_robots:
        :param size_of_field:
        """
        self._robot_class = robot_class
        self._number_of_teams = number_of_teams
        self._number_of_robots = number_of_robots
        self._size_of_field = size_of_field
        self._robots = [list() for _ in range(self._number_of_teams)]
        self.team_CS_rotations = [0, np.pi]
        self.team_starting_points = [(-7,3), (-7,-3), (-3,3), (-3, -3), (-8, 0)]
        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                self._robots[team].append(
                    self._robot_class(*self.team_starting_points[player_id], cord_system_rot=self.team_CS_rotations[team]))

        self.ball = ball_model(0, 0)

    def step(self, actions_per_team_per_player):
        """
        Execute the simulation step for all components
        In this version the steps are sequential -> team 1 players, team 2 players, ball
        :param actions_per_team_per_player:
        :return:
        """
        other_players_actions = []
        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                action = actions_per_team_per_player[team][player_id]
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

    def get_positions_for_visualizer(self):
        """
        Get simulation state to update visualizer,
        also handle the conversion to the visualizer format (FIXME: should be in the visualizer not here)
        :return:
        """
        ball = self.ball.get_position()
        ball = np.array([ball[0], -ball[1]])  # FIXME: minus is due to inverted axis in pygames
        team_0, team_1 = [], None
        for player_id in range(self._number_of_robots):
            tp0_x, tp0_y = self._robots[0][player_id].get_position_components_wcs()
            team_0.append([tp0_x, -tp0_y])                                             # FIXME: minus is due to inverted axis in pygames
            #tp1_x, tp1_y = self._robots[01[player_id].get_position_components_wcs() # TODO: add for other team
        team_0 = np.array(team_0) + np.array(self._size_of_field) / 2
        ball = ball + np.array(self._size_of_field) / 2
        return team_0 * 50, team_1, ball * 50  # FIXME: 50 is scalar factor to match the visualization size

    def get_positions(self):
        """
        FIXME: should return positions of all robots in teams - currently it is fixed for single team
        :return:
        """
        x_pos_1, x_pos_2 = [], []
        y_pos_1, y_pos_2 = [], []
        for player_id in range(self._number_of_robots):
            x, y = self._robots[0][player_id].get_position_components_wcs()
            x_pos_1.append(x), y_pos_1.append(y)
            #x, y = self._robots[1][player_id].get_position_components_wcs()
            #x_pos_2.append(x), y_pos_2.append(y)

        return np.array([x_pos_1, y_pos_1])

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
        safe_collision_threshold = 1.2  # FIXME
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

    def check_if_ball_in_net(self):
        raise NotImplementedError


class TestGameSimulation:
    @staticmethod
    def test_game_initialization():
        from GameEngine.robot_model import RobotBasicModel
        from GameEngine.ball_model import BallBasicModel
        game = GameSimulator(RobotBasicModel, BallBasicModel, number_of_robots=5)
        pass

    @staticmethod
    def test_non_collision_step():
        from GameEngine.robot_model import RobotBasicModel
        from GameEngine.ball_model import BallBasicModel
        import matplotlib.pyplot as plt
        number_of_robots = 5
        game = GameSimulator(RobotBasicModel, BallBasicModel, number_of_robots=number_of_robots)

        def get_robots_positions(game_):
            x_pos = []
            y_pos = []
            for player in game_._robots[0]:
                x, y = player.get_position_components_wcs()
                x_pos.append(x), y_pos.append(y)
            return x_pos, y_pos

        simulation_steps = 1000
        history = {'players_x': [], 'players_y': [], 'ball_x': [], 'ball_y': []}
        actions = [[(0.3, 0.35) for _ in range(number_of_robots)]]
        for i in range(simulation_steps):
            game.step(actions)
            xx, yy = get_robots_positions(game)
            history['players_x'].append(xx)
            history['players_y'].append(yy)
            bx, by = game.ball.get_position()
            history['ball_x'].append(bx)
            history['ball_x'].append(by)

        pos_x = np.array(history['players_x'])
        pos_y = np.array(history['players_y'])
        plt.plot(pos_x, pos_y)
        plt.gca().set_aspect('equal')
        pass

    @staticmethod
    def test_game_collision_steps_with_visualizer():
        from GameEngine.robot_model import RobotBasicModel
        from GameEngine.ball_model import BallBasicModel
        import matplotlib.pyplot as plt
        from GameEngine.visualizer import BasicVisualizer

        number_of_robots = 5
        visualizer = BasicVisualizer(None, number_of_players=number_of_robots)
        game = GameSimulator(RobotBasicModel, BallBasicModel, number_of_robots=number_of_robots)

        def get_robots_positions(game_):
            x_pos = []
            y_pos = []
            for player in game_._robots[0]:
                x, y = player.get_position_components_wcs()
                x_pos.append(x), y_pos.append(y)
            return x_pos, y_pos

        simulation_steps = 3000
        history = {'players_x': [], 'players_y': [], 'ball_x': [], 'ball_y': []}
        actions = [[(0.3, 0.35) for _ in range(number_of_robots)]]
        game.ball._x_vel, game.ball._y_vel = 0.01, 0.005
        for i in range(simulation_steps):
            game.step(actions)
            visualizer.send_game_state(*game.get_positions_for_visualizer())
            visualizer.display()

            xx, yy = get_robots_positions(game)
            history['players_x'].append(xx)
            history['players_y'].append(yy)
            bx, by = game.ball.get_position()
            history['ball_x'].append(bx)
            history['ball_y'].append(by)

        pos_x = np.array(history['players_x'])
        pos_y = np.array(history['players_y'])
        plt.plot(pos_x, pos_y)
        plt.plot(history['ball_x'], history['ball_y'])
        plt.gca().set_aspect('equal')
        pass

if __name__ == "__main__":
    #TestGameSimulation.test_game_initialization()
    #TestGameSimulation.test_game_collision_steps_with_visualizer()
    TestGameSimulation.test_game_collision_steps_with_visualizer()







