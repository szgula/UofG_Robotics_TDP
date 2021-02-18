from game_interfaces.msg import TeamPosition, PlayerCommand, TeamCommand
from robocup_ws.src.Planner.src.goalkeeper_controller import TempGoalkeeperController
import logging


class TeamMaster:

    def __init__(self, team_id):
        self.team_id = team_id
        self.team_position = None
        self.position_updated = False
        self.actions_planned = True
        self.actions = [PlayerCommand(0,0,0) for _ in range(5)]

        self.striker_left_idx = 0
        self.striker_right_idx = 1
        self.defence_left_idx = 2
        self.defence_righr_idx = 3
        self.goalkeeper_idx = 4

        self.goalkeeper_logic = TempGoalkeeperController()

    def update_game_state(self, team_position: TeamPosition):
        """
        Collect information about game state (# of goals, game time...) and players (inner state, etc)
        :return:
        """
        self.team_position = team_position
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

        self.actions = [PlayerCommand(0,0,0) for _ in range(5)]

        self.actions[self.goalkeeper_idx] = self.goalkeeper_logic.get_action(
            self.team_position.players_positions_efcs[self.goalkeeper_idx], self.team_position.ball_pos_efcs
        )
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
