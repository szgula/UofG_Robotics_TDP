from robot_control import Robot, Goal
from game_interfaces.msg import PlayerCommand
from brachistochrone import cycloid

class PlayerController(Robot):
    def __init__(self):
        pass

    def get_action(self, my_pos_efcs, ball_pos_efcs):
        x, y, T, Vr, Vl = cycloid(my_pos_efcs,ball_pos_efcs,2)
        return PlayerCommand(Vl, Vr, 0)
