class GameMaster:
    def __init__(self, robot_class, number_of_teams: int = 1, number_of_robots: int = 1, size_of_field: tuple = (18, 12)):
        """
        :param robot_class:
        :param number_of_teams:
        :param number_of_robots:
        :param size_of_field:
        """
        self._robot_class = robot_class
        self._number_of_teams = number_of_teams
        self._number_of_robots = number_of_robots
        self._size_of_field = size_of_field

        # Ego field coordinate system: located in the middle of the field,
        # positive X towards opponent's goal
        # positive Y 90deg rotated counterclockwise from X axis

        # Base simulation should be synchronous - the simulation waits for all inputs and is each step is triggered by
        # step() method.

    def reset(self):
        """
        Reset the play
        :return:
        """
        pass

    def update_robot_actions(self, team_id: int, actions: tuple):
        """
        Update the robots actions - aggregate from all agents
        :return:
        """
        pass

    def step(self):
        """
        Execute the simulation step
        :return:
        """
        pass

    def get_game_state(self):
        """
        Interface for the visualization
        :return:
        """
        pass
