import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer, TeamMaster
from goalkeeper_controller import Team1GoalkeeperController
from stay_in_place_controller import NullController


class TeamMaster0(TeamMaster):
    def __init__(self):
        team_id = 0
        super().__init__(team_id)
        self.goalkeeper_logic = Team1GoalkeeperController(0)
        self.striker_left_logic = NullController()
        self.striker_right_logic = NullController()
        self.defence_left_logic = NullController()
        self.defence_right_logic = NullController()
        self.players_logic_was_updated = True


if __name__ == "__main__":
    team_mater_0 = TeamMaster0()
    TMS = TeamMasterServer(team_mater_0)
