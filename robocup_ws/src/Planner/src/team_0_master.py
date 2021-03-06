import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer, TeamMaster
from goalkeeper_controller import Team1GoalkeeperController
from stay_in_place_controller import NullController
from decision_tree import DecisionTree
from player_controller import PlayerController


class TeamMaster0(TeamMaster):
    def __init__(self):
        team_id = 0
        super().__init__(team_id)
        self.goalkeeper_logic = Team1GoalkeeperController(self.team_id)
        self.striker_left_logic = NullController()
        self.striker_right_logic = NullController()
        self.defence_left_logic = NullController()
        self.defence_right_logic = NullController()
        self.players = {self.defence_left_logic: "DEFEND",
                        self.defence_right_logic: "DEFEND",
                        self.striker_left_logic: "ATTACK",
                        self.striker_right_logic:"ATTACK" }
        self.tree = None
        self.players_logic_was_updated = True

    def plan(self):
        super(TeamMaster0, self).plan()
        player_id = 0
        for player, mode in self.players.items():
            player = PlayerController(player_id)
            self.tree = DecisionTree(self.team_id, pc = player)
            self.actions[player_id] = self.tree.plan(self.team_position,self.opponents_position, player_id, mode)
            player_id+=1


if __name__ == "__main__":
    team_mater_0 = TeamMaster0()
    TMS = TeamMasterServer(team_mater_0)
