#FIXME: imports etc, migrated form game_simulator.py


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