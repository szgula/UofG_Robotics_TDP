from GameEngine.robot_model import RobotBasicModel
from GameEngine.ball_model import BallBasicModel, BallActions
# import matplotlib.pyplot as plt
from GameEngine.visualizer import BasicVisualizer
from GameEngine.game_simulator import GameSimulator
import logging
from time import sleep


class Game_Master:
    def __init__(self):
        """
        :param robot_class:
        :param number_of_teams:
        :param number_of_robots:
        :param size_of_field:
        """
        # Ego field coordinate system: located in the middle of the field,
        # positive X towards opponent's goal
        # positive Y 90deg rotated counterclockwise from X axis
        self.number_of_robots = 1
        self.number_of_teams = 1
        self.visualizer = BasicVisualizer(None, number_of_players=self.number_of_robots)
        self.simulator = None
        self.reset()

        self.full_game_length = 1000
        self.game_current_step = 0
        self.goals = [0, 0]

        self.action_buffer = [None] * self.number_of_teams
        self.action_updated = [False] * self.number_of_teams

    def reset(self):
        """
        Reset the play
        :return:
        """
        self.simulator = GameSimulator(RobotBasicModel, BallBasicModel,
                                       number_of_robots=self.number_of_robots,
                                       number_of_teams=self.number_of_teams)
        pass

    def update_robot_actions(self, team_id: int, actions: tuple):
        """
        Update the robots actions - aggregate from all agents
        :return:
        """
        self.action_updated[team_id] = True
        self.action_buffer[team_id] = actions

    def step(self):
        """
        Execute the simulation step
        :return:
        """
        self.game_current_step += 1
        logging.info(f'current game step {self.game_current_step}')
        if not all(self.action_updated):
            logging.warning("Some team have not updated the action")
        self.simulator.step(self.action_buffer)
        self.visualizer.send_game_state(*self.simulator.get_positions_for_visualizer())
        self.visualizer.display()

        if self.game_current_step >= self.full_game_length:
            self.end_game()

    def end_game(self):
        logging.info(f"End of game with result {self.goals}")

    def get_game_state(self):
        """
        Interface for the visualization
        :return:
        """
        return self.goals, self.game_current_step


class TestGameMaster:
    @staticmethod
    def test_game_master_initialization():
        pass

    @staticmethod
    def test_game_with_simple_actions():
        pass


if __name__ == "__main__":
    game_master = Game_Master()
    actions = [(0, 0, BallActions.KICK)]
    kick_done = False

    #The simulation begins and lasts for full_game_length
    key = "s"
    for i in range(game_master.full_game_length):
        game_master.update_robot_actions(0, actions)
        game_master.step()
        if key != "c":
            key = input("Press Enter to continue...")
        if i == 0:
            sleep(0.75)
