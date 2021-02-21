from tqdm import tqdm
import logging
import rospy
from game_interfaces.srv import SimulationUpdate, SimulationUpdateRequest, TeamUpdate, TeamUpdateRequest
from game_interfaces.msg import PlayerCommand, TeamCommand


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


class GameMasterClient(BaseGameMaster):
    def __init__(self):
        super().__init__()
        rospy.init_node('game_master_client')
        rospy.wait_for_service(r'game_engine/game_simulation')
        rospy.wait_for_service(r'agents/team_master_0')
        rospy.wait_for_service(r'agents/team_master_1')
        try:
            self.sim_server = rospy.ServiceProxy(r'game_engine/game_simulation', SimulationUpdate)
            self.team0_server = rospy.ServiceProxy(r'agents/team_master_0', TeamUpdate)
            self.team1_server = rospy.ServiceProxy(r'agents/team_master_1', TeamUpdate)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            raise e

    def send_sim_update_request(self, team_actions):
        rq = SimulationUpdateRequest(True, False, team_actions)
        resp = self.sim_server(rq)
        return resp.status, resp

    def send_teams_update_request(self, sim_response):
        team0_message = TeamUpdateRequest(True,
                                          sim_response.teams_position[0],
                                          sim_response.teams_position[1],
                                          self.goals[0], self.goals[1],
                                          self.game_current_step, self.game_current_step / self.full_game_length)
        team1_message = TeamUpdateRequest(True,
                                          sim_response.teams_position[1],
                                          sim_response.teams_position[0],
                                          self.goals[1], self.goals[0],
                                          self.game_current_step, self.game_current_step / self.full_game_length)

        resp_team0 = self.team0_server(team0_message)
        resp_team1 = self.team1_server(team1_message)
        return resp_team0.status and resp_team1.status, resp_team0, resp_team1


if __name__ == "__main__":
    GMC = GameMasterClient()

    actions = [TeamCommand(i, [PlayerCommand(0, 0, 0) for _ in range(5)]) for i in range(2)]
    for i in tqdm(range(GMC.full_game_length)):
        status, sim_output = GMC.send_sim_update_request(actions)
        status, m_team_0, m_team_1 = GMC.send_teams_update_request(sim_output)
        actions = [m_team_0.team_commands, m_team_1.team_commands]
