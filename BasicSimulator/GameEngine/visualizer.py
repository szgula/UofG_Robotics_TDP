import numpy as np
import pygame
import logging

#from GameEngine.game_master import BaseSimulator

# TODO: add unittests, smoke test, and functional tests
# TODO: Write documentation


class BasicVisualizer:

    def __init__(self, simulator: None, number_of_players: int = 2, field_size: tuple = (10, 6),
                 display_scale: int = 100):
        #self._simulator = simsimulatorulator
        self._number_of_players = number_of_players
        self._position_dim = 3  # x, y, heading
        self._field_size = np.array(field_size)
        self._display_size = self._field_size * display_scale
        self.scale = display_scale
        self._init_display()

        self._temp_initialized_data = False
        self._data_provided = False
        self._data_provided_buffer = ()

    def __del__(self):
        pygame.quit()

    def _init_display(self):
        """
        Initialize the pygame window and setup the flag of running visualizer
        :return:
        """
        pygame.init()
        self.screen = pygame.display.set_mode(self._display_size)
        self.vis_running = True

    def _exit_display(self):
        pygame.quit()
        self.vis_running = False

    def display(self):
        """
        Display callback function, this function should handle window events (close event etc) and call draw functions
        for players and ball
        :return:
        """
        if not self.vis_running:
            logging.warning('The visualization was closed')
        state_players_1, state_players_2, ball = self.accrue_data_from_simulatior()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._exit_display()
        self.screen.fill((255, 255, 255))
        for pos in state_players_1:
            pygame.draw.circle(self.screen, (0, 0, 255), (pos[:2]*self.scale).astype(int), 7)
        pygame.draw.circle(self.screen, (255, 0, 0), (ball*self.scale).astype(int), 5)
        #Drawing goal post one
        #This draws the goalpost
        pygame.draw.line(self.screen, (0, 255, 0),
                         (5, 50 + self._display_size[1] / 2),
                         (5, -50 + self._display_size[1] / 2), 3)
        #Drawing goal post two
        pygame.draw.line(self.screen, (0, 255, 0),
                         (self._display_size[0] - 5, 50 + self._display_size[1] / 2),
                         (self._display_size[0] - 5, -50 + self._display_size[1] / 2), 3)
        pygame.draw.circle(self.screen, (77, 200, 77),
                         (self._display_size[0] -5, self._display_size[1]/2), 5) 
        pygame.display.flip()

    def accrue_data_from_simulatior(self, team_1=None, team_2=None, ball=None) -> (tuple, tuple, tuple):  # FIXME: rename to send data to visualizer
        """
        Accrue data from game simulation - players and ball positions, # of goals, game time etc
        :return:
        """
        if not self._data_provided:
            return self._temp_generate_random_data_from_simulation()
        else:
            self._data_provided = False
            return self._data_provided_buffer


    def send_game_state(self, team_1, team_2, ball):
        self._data_provided = True
        self._data_provided_buffer = (team_1, team_2, ball)

    def _temp_generate_random_data_from_simulation(self):
        # generate random positions
        if not self._temp_initialized_data:
            random_position_team_1 = np.random.random((self._number_of_players, self._position_dim))
            random_position_team_1 = random_position_team_1 * np.array([*self._display_size, 2 * np.pi])
            random_position_team_1[:, 2] -= np.pi
            ball_position = np.random.random(2) * self._display_size
            self._last_pos = random_position_team_1, None, ball_position
            self._temp_initialized_data = True
            self._temp_vel_vector = (1, 1, 0)
            self._temp_counter = 0
        else:
            self._temp_counter += 1
            if self._temp_counter > 500:
                self._temp_counter = 0
                self._temp_vel_vector = (np.random.random((self._number_of_players, self._position_dim)) - 0.5) * 2

            random_position_team_1, _, ball_position = self._last_pos
            random_position_team_1 += (np.random.random((self._number_of_players, self._position_dim)) - 0.5) * [5, 5,
                                                                                                                 0.1]
            random_position_team_1 += self._temp_vel_vector
            random_position_team_1[:, 0] = random_position_team_1[:, 0].clip(min=0, max=self._display_size[0])
            random_position_team_1[:, 1] = random_position_team_1[:, 1].clip(min=0, max=self._display_size[1])

        return random_position_team_1, None, ball_position


if __name__ == "__main__":
    my_vis = BasicVisualizer(None)
    while my_vis.vis_running:
        my_vis.display()
    pass
