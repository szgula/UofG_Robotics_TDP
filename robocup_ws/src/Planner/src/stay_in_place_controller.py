from robot_control import Robot
from game_interfaces.msg import PlayerCommand


class NullController(Robot):
    def __init__(self):
        pass

    def get_action(self, my_pos_efcs, ball_pos_efcs, team_positions_wcs=None, opponents_positions_wcs=None):
        l_rpm = 0
        r_rpm = 0
        action = 0
        return PlayerCommand(l_rpm, r_rpm, action)