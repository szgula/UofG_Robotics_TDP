from abc import ABC, abstractmethod
from enum import Enum


class Goal(Enum):
    NO = 0
    ChaseBall = 1
    ScoreGoal = 2


class Robot(ABC):

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_action(self, goal):
        """
        Generate action based on the goal, state etc.
        :param goal:
        :return:
        """
        pass

    @abstractmethod
    def accrue_sensors_data(self):
        """
        Get information from robot/model an convert it to the robot state
        :return:
        """
        pass

