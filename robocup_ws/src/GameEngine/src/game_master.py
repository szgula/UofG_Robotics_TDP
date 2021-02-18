import sys, os
import inspect
#FIXME: this is awful! @Omar - I think it is not needed, find better way
sys.path.append('../../')
cwd = os.getcwd()
sys.path.append(cwd)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from tqdm import tqdm
import logging
import rospy
from game_interfaces.srv import SimulationUpdate, SimulationUpdateRequest
from game_interfaces.msg import TeamCommand


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
        self.simulator = None

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
        raise NotImplementedError

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


class GameMasterClient:
    def __init__(self):
        rospy.init_node('game_master_client')
        rospy.wait_for_service(r'game_engine/game_simulation')
        try:
            self.server = rospy.ServiceProxy(r'game_engine/game_simulation', SimulationUpdate)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            raise e

    def send_update_request(self, team_actions):
        rq = SimulationUpdateRequest(True, False, team_actions)
        resp = self.server(rq)
        return resp.status, resp



if __name__ == "__main__":
    from robocup_ws.src.Planner.src.team_master import TeamMaster
    team0_master, team1_master = TeamMaster(0), TeamMaster(1)
    GMC = GameMasterClient()

    actions = [team0_master.distribute_goals_to_players(), team1_master.distribute_goals_to_players()]
    for i in tqdm(range(5000)):
        status, message = GMC.send_update_request(actions)

        # TODO: all 3 methods can be replace with a single function
        team0_master.update_game_state(message.teams_position[0])
        team0_master.plan()
        team0_actions = team0_master.distribute_goals_to_players()

        team1_master.update_game_state(message.teams_position[1])
        team1_master.plan()
        team1_actions = team0_master.distribute_goals_to_players()

        actions = [team0_actions, team1_actions]
