from abc import ABC, abstractmethod


class Action(ABC):

    def __init__(self, data: list):
        self._data = data

    @abstractmethod
    def do_algorithm(self, data: list):
        pass
