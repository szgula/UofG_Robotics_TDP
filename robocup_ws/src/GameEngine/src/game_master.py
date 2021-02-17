import sys, os
sys.path.append('../../')
cwd = os.getcwd()
sys.path.append(cwd)
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

print(f'cwd = {cwd}')
print(f'path = {sys.path}')

from tqdm import tqdm
from src.robot_model import RobotBasicModel
from src.ball_model import BallBasicModel, BallActions
# import matplotlib.pyplot as plt
from src.visualizer import BasicVisualizer
from src.game_simulator import GameSimulator
import logging


class BaseGameMaster:
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
        self.number_of_robots = 5
        self.number_of_teams = 2
        self.visualizer = BasicVisualizer(None, number_of_players=self.number_of_robots)
        self.simulator = None
        self.reset()

        self.full_game_length = 30000
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

    def update_goal_counter(self, goal_status):
        if goal_status != 0:
            team_id = goal_status - 1
            self.goals[team_id] += 1


ROS = True
if __name__ == "__main__" and ROS:
    import rospy
    from game_interfaces.srv import SimulationUpdate, SimulationUpdateRequest
    from game_interfaces.msg import TeamCommand
    pass


class GameMasterClient:
    def __init__(self):
        rospy.init_node('game_master_client')
        rospy.wait_for_service(r'game_engine/game_simulation')
        try:
            self.server = rospy.ServiceProxy(r'game_engine/game_simulation', SimulationUpdate)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            raise e

    def send_update_request(self, actions_list):
        team_actions = self.convert_action_list_to_team_commands(actions_list)
        rq = SimulationUpdateRequest(True, False, team_actions)
        resp = self.server(rq)
        return resp.status

    @staticmethod
    def convert_action_list_to_team_commands(action_list):
        out = []
        for team_id, team_actions in enumerate(action_list):
            tc = TeamCommand()
            tc.team_id = team_id
            for player_idx, (lw, rw) in enumerate(team_actions):
                tc.players_commands[player_idx].left_rpm = lw
                tc.players_commands[player_idx].right_rpm = rw
                tc.players_commands[player_idx].extra_action = 0
            out.append(tc)
        return out


if __name__ == "__main__" and ROS:
    GMC = GameMasterClient()

    actions = [[(0.97, 1.0), (1.0, 0.97), (-0.7, -1.0), (1.3, 1.05), (0.99, 1.0)], [(0,0), (0,0), (0,0), (0,0), (1.1, 1.1)]]
    for i in tqdm(range(5000)):
        GMC.send_update_request(actions)

