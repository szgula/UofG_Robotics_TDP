import numpy as np
import math
import rospy
from robocup_control.srv import ActionServices, ActionServicesResponse
from GameEngine.src.game_master import BaseGameMaster
from GameEngine.src.game_simulator import *
from Planner.BasicCommonActions.BasicCommonActionsHandler import BasicCommonActionsHandler
from Planner.BasicCommonActions.ChaseBallAction import ChaseBallAction
from Planner.BasicCommonActions.DoNothingAction import DoNothingAction
from Planner.BasicCommonActions.RotateToPointAction import RotateToPointAction
from GameEngine.src.ball_model import BallBasicModel
from GameEngine.src.robot_model import RobotBasicModel
from Planner.robot_control import Robot, Goal


class GoalkeeperController(Robot):

    def __init__(self, goalkeeper_model: RobotBasicModel, ball_model: BallBasicModel):
        self._goalkeeper = goalkeeper_model
        self._ball = ball_model
        self._lookhead = 10
        self._bscActHander = BasicCommonActionsHandler(DoNothingAction([goalkeeper_model]))

    def get_action(self, goal) -> tuple:
        if goal == Goal.ChaseBall:
            self._bscActHander.action = ChaseBallAction([self._goalkeeper, self._ball])
        elif goal == Goal.RotateToPoint:
            self._bscActHander.action = RotateToPointAction([self._goalkeeper, self._ball])
        return self._bscActHander.handle()

    def accrue_sensors_data(self):
        pass

def handle_actions(req):
    robot_model = RobotBasicModel(req.player_x,req.player_y)
    ball_model = BallBasicModel(req.ball_x,req.ball_y)
    team01_goalkeeper = GoalkeeperController(robot_model,ball_model)
    actions = team01_goalkeeper.get_action(Goal.ChaseBall)
    # print(actions)
    return ActionServicesResponse(actions[1],actions[0])

def actions_server():
    rospy.init_node("actions_server")
    s = rospy.Service("actions_return",ActionServices,handle_actions)
    print("Ready to handle actions.")
    rospy.spin()
if __name__ == "__main__":
    actions_server()