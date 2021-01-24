import numpy as np
from GameEngine.robot_model import RobotModel
from GameEngine.ball_model import BallModel
from GameEngine.collisions import CollisionTypes


class GameSimulator:

    def __init__(self, robot_class: RobotModel, ball_model: BallModel, number_of_teams: int = 1, number_of_robots: int = 1,
                 size_of_field: tuple = (18, 12), visualizer=None):
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
        self._visualizer = visualizer

        #init game_masters

    def step(self, actions_per_team_per_player):
        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                action = actions_per_team_per_player[team][player_id]
                any_collision, *collisions = self.check_for_collisions(team, player_id)

                if not any_collision:
                    self._robots[team][player_id].step(*action)
                else:
                    self._robots[team][player_id].collision_step(*collisions, *action)
        self.ball.step()
        if self._visualizer:
            self._visualizer.send_game_state(*self.get_positions_for_visualizer())
            self._visualizer.display()

    def get_positions_for_visualizer(self):
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

    #def check_ball_collisions(self, ):

    def check_for_collisions(self, team_id, player_id):
        robot = self._robots[team_id][player_id]
        wall_collision = self.check_wall_player_collisions(robot)
        ball_collision = self.check_ball_player_collision(robot)
        players_collision, collisions_with = self.check_players_player_collisions(robot, player_id, team_id)
        any_collision = True
        if wall_collision == CollisionTypes.NO and ball_collision == CollisionTypes.NO and players_collision == CollisionTypes.NO:
            any_collision = False
        return any_collision, wall_collision, ball_collision, players_collision, collisions_with

    def check_wall_player_collisions(self, player):
        x, y = player.get_position_components_wcs()
        vert, horiz = False, False
        if x >= self._size_of_field[0] / 2 or x <= -self._size_of_field[0] / 2:
            horiz = True
        if y >= self._size_of_field[1] / 2 or y <= -self._size_of_field[1] / 2:
            vert = True

        if horiz and vert: collision_type = CollisionTypes.WALL_CORNER
        elif horiz: collision_type = CollisionTypes.WALL_HORIZONTAL
        elif vert: collision_type = CollisionTypes.WALL_VERTICAL
        else: collision_type = CollisionTypes.NO
        return collision_type

    def check_ball_player_collision(self, player: RobotModel):
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

    def check_players_player_collisions(self, player, this_player_id, this_team_id):
        safe_collision_threshold = 1.2  # FIXME
        p_x, p_y = player.get_position_components_wcs()
        R = player.radius * 2 * safe_collision_threshold
        collisions_list = []

        for team in range(self._number_of_teams):
            for player_id in range(self._number_of_robots):
                if team == this_team_id and player_id == this_player_id:
                    continue
                o_x, o_y = self._robots[team][player_id].get_position_components_wcs()
                collision = np.hypot(p_x - o_x, p_y - o_y) <= R
                if collision:
                    collisions_list.append((team, player_id))
        collision = CollisionTypes.PLAYER if len(collisions_list) else CollisionTypes.NO
        return collision, collisions_list


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
        game = GameSimulator(RobotBasicModel, BallBasicModel, number_of_robots=number_of_robots, visualizer=visualizer)

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
        game.ball._x_vel, game.ball._y_vel = 0.01, 0.005
        for i in range(simulation_steps):
            game.step(actions)
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







