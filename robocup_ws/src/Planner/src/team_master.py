from abc import ABC, abstractmethod
from game_interfaces.msg import PlayerCommand, TeamCommand
from game_interfaces.srv import TeamUpdate, TeamUpdateRequest, TeamUpdateResponse
#import sys, os
#sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from goalkeeper_controller import TempGoalkeeperController
import logging
import rospy


class TeamMaster(ABC):

    @abstractmethod
    def __init__(self, team_id):
        self.team_id = team_id
        self.team_position = None
        self.opponents_position = None
        self.position_updated = False
        self.actions_planned = True

        self.time_percentage = 0
        self.goals_our = 0
        self.goals_opponent = 0

        self.actions = [PlayerCommand(0, 0, 0) for _ in range(5)]

        self.striker_left_idx = 0
        self.striker_right_idx = 1
        self.defence_left_idx = 2
        self.defence_right_idx = 3
        self.goalkeeper_idx = 4
        self.goalkeeper_logic = None
        self.striker_left_logic = None
        self.striker_right_logic = None
        self.defence_left_logic = None
        self.defence_right_logic = None
        self.players_logic_was_updated = False
    """
        ADD FOLLOWING TO YOUR init method:
        
        super().__init__(team_id)
        self.goalkeeper_logic = TempGoalkeeperController()
        self.striker_left_logic = 
        self.striker_right_logic = 
        self.defence_left_logic = 
        self.defence_right_logic = 
        self.players_logic_was_updated = True
        """

    def update_game_state(self, team_update_request: TeamUpdateRequest):
        """
        Collect information about game state (# of goals, game time...) and players (inner state, etc)
        :return:
        """
        self.team_position = team_update_request.players_positions
        self.opponents_position = team_update_request.opponents_positions
        self.time_percentage = team_update_request.time_percentage
        self.goals_our = team_update_request.team_goals
        self.goals_opponent = team_update_request.opponents_goals
        self.position_updated = True
        self.actions_planned = False

    def plan(self):
        """
        Generate short/mid term tactic/strategy
        :return:
        """
        if not self.position_updated:
            raise logging.waringn("The simulation state has not beer updated, please run the update_game_state() method before")
            return
        if not self.players_logic_was_updated:
            raise logging.waringn("The players logic was not updated - please do so before planning")
            return

        self.actions = [PlayerCommand(0, 0, 0) for _ in range(5)]

        self.actions[self.goalkeeper_idx] = self.goalkeeper_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs)
        self.actions[self.striker_left_idx] = self.striker_left_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs)
        self.actions[self.striker_right_idx] = self.striker_right_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs)
        self.actions[self.defence_left_idx] = self.defence_left_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs)
        self.actions[self.defence_right_idx] = self.defence_right_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs)
        self.actions_planned = True

    def distribute_goals_to_players(self):   # TODO: how about to rename 'goals -> commends'
        """
        Send action goals to players
        :return:
        """
        if not self.actions_planned:
            logging.warning("The new actions were not generated - please run plan() method before")
        self.position_updated = False
        self.actions_planned = False
        message = TeamCommand(self.team_id, self.actions)
        return message


class TeamMasterServer:
    def __init__(self, team_master_object):
        self.team_master = team_master_object
        rospy.init_node(f'team_master_{self.team_master.team_id}_server')
        service_name = rf'agents/team_master_{self.team_master.team_id}'
        s = rospy.Service(service_name, TeamUpdate, self.handle_team_update_call)
        print(f"Team master {self.team_master.team_id} ready to control the team. (name={service_name})")
        rospy.spin()

    def handle_team_update_call(self, simulation_request):
        status = True
        self.team_master.update_game_state(simulation_request)
        self.team_master.plan()
        actions = self.team_master.distribute_goals_to_players()
        response = TeamUpdateResponse(status, actions)

        return response

