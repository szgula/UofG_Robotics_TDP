from abc import ABC, abstractmethod


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

