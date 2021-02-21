import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer


if __name__ == "__main__":
    TMS = TeamMasterServer(1)
