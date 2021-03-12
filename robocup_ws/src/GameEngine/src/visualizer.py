import numpy as np
import pygame
import logging
from pygame.locals import *


# TODO: add unittests, smoke test, and functional tests
# TODO: Write documentation


class BasicVisualizer:

    def __init__(self, simulator: None, number_of_players: int = 5, field_size: tuple = (10, 6),
                 display_scale: int = 100):
        pygame.init()
        self.font = pygame.font.SysFont('Arial', 16)
        self._number_of_players = number_of_players
        self._position_dim = 3  # x, y, heading
        self._field_size = np.array(field_size)
        self._display_size = self._field_size * display_scale
        self._display_size_width = self._field_size[0] * display_scale
        self._display_size_height = self._field_size[1] * display_scale
        self.scale = display_scale
        self._init_display()

        self._temp_initialized_data = False
        self._data_provided = False
        self._data_provided_buffer = ()
        self.fps = 500
        self.fclock = pygame.time.Clock()
        self._robo_radius = 10
        self._field_line_color = (255, 255, 255)
        self._field_color = (55, 170, 80)
        self._robo_dirc_color = (255, 0, 0)
        self._field_line_width = 3
        self._center_circle_radius = 1 * display_scale
        self._goal_area_width = 1.5 * display_scale
        self._goal_area_height = 3 * display_scale
        self._offside_area_radius = 0.8 * display_scale
        self._offside_center_distance = 1.1 * display_scale
        self._margin = 0.3 * display_scale
        self._gate_height = 2 * display_scale
        self._gate_color = (170, 170, 170)

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
        state_players_1, state_players_2, ball, score = self.accrue_data_from_simulator()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._exit_display()
        self.screen.fill(self._field_color)
        # center circle
        center_circle_size = (self._display_size_width / 2 - self._center_circle_radius,
                              self._display_size_height / 2 - self._center_circle_radius,
                              2 * self._center_circle_radius,
                              2 * self._center_circle_radius)
        pygame.draw.ellipse(self.screen, self._field_line_color, center_circle_size, self._field_line_width)
        # center field rect
        center_field_size = (self._margin, self._margin, self._display_size_width - self._margin * 2,
                             self._display_size_height - self._margin * 2)
        pygame.draw.rect(self.screen, self._field_line_color, center_field_size, self._field_line_width)
        # middle line
        pygame.draw.line(self.screen, self._field_line_color, [self._display_size_width / 2, self._margin],
                         [self._display_size_width / 2, self._display_size_height - self._margin],
                         self._field_line_width)
        # left goal area
        left_goal_area_size = (self._margin, self._display_size_height / 2 - self._goal_area_height / 2,
                               self._goal_area_width, self._goal_area_height)
        pygame.draw.rect(self.screen, self._field_line_color, left_goal_area_size, self._field_line_width)
        # right goal area
        right_goal_area_size = ((self._display_size_width - self._margin - self._goal_area_width),
                                (self._display_size_height / 2 - self._goal_area_height / 2),
                                self._goal_area_width,
                                self._goal_area_height)
        pygame.draw.rect(self.screen, self._field_line_color, right_goal_area_size, self._field_line_width)
        # left offside area
        left_offside_area_size = (self._margin + self._offside_center_distance - self._offside_area_radius,
                                  (self._display_size_height / 2 - self._offside_area_radius),
                                  2 * self._offside_area_radius, 2 * self._offside_area_radius)
        r = self._offside_area_radius
        l = self._goal_area_width - self._offside_center_distance
        offside_area_theta = np.arccos(l / r)
        pygame.draw.arc(self.screen, self._field_line_color, left_offside_area_size, -1 * offside_area_theta,
                        offside_area_theta,
                        self._field_line_width)
        # right offside area
        right_offside_area_size = (
        (self._display_size_width - self._margin - self._offside_center_distance - self._offside_area_radius),
        (self._display_size_height / 2 - self._offside_area_radius),
        2 * self._offside_area_radius, 2 * self._offside_area_radius)
        r = self._offside_area_radius
        l = self._goal_area_width - self._offside_center_distance
        offside_area_theta = np.arccos(l / r)
        pygame.draw.arc(self.screen, self._field_line_color, right_offside_area_size, -1 * offside_area_theta + np.pi,
                        offside_area_theta + np.pi,
                        self._field_line_width)
        # left gate
        left_gate_size = (self._margin, self._display_size_height / 2 - self._gate_height / 2,
                          self._goal_area_width / 3, self._gate_height)
        pygame.draw.rect(self.screen, self._field_line_color, left_gate_size, self._field_line_width)
        # right gate
        right_gate_size = ((self._display_size_width - self._margin - self._goal_area_width / 3 - 1),
                           self._display_size_height / 2 - self._gate_height / 2,
                           self._goal_area_width / 3, self._gate_height)
        pygame.draw.rect(self.screen, self._field_line_color, right_gate_size, self._field_line_width)
        # ball
        pygame.draw.circle(self.screen, (255, 0, 0), (ball * self.scale).astype(float), 5)
        # ball_point = (ball * self.scale).astype(float)
        # pygame.draw.rect(self.screen, (0, 0, 255), (ball_point[0] - 50, ball_point[1] - 50, 100, 100), 1)

        player_id = 0
        for pos in state_players_1:
            pygame.draw.circle(self.screen, (173, 216, 230), (pos[:2] * self.scale).astype(float), self._robo_radius)
            self.draw_direction_arrow_and_id_indicator(pos, player_id)
            player_id += 1
        player_id = 0
        for pos in state_players_2:
            pygame.draw.circle(self.screen, (0, 255, 0), (pos[:2] * self.scale).astype(float), self._robo_radius)
            self.draw_direction_arrow_and_id_indicator(pos, player_id)
            player_id += 1
        font = pygame.font.SysFont(None, 48)
        img = font.render(f'{score[0]} - {score[1]}', True, (255, 0, 0))
        self.screen.blit(img, (self._display_size[0] / 2 - 30, 0))
        pygame.display.flip()
        self.fclock.tick(self.fps)

    def draw_direction_arrow_and_id_indicator(self, pos, player_id):
        start = np.array(pos[:2] * self.scale).reshape(2, 1)
        start_change = np.array(
            [0.5 * self._robo_radius * np.cos(pos[2:]), -0.5 * self._robo_radius * np.sin(pos[2:])]).reshape(2, 1)
        end_change = np.array(
            [1.2 * self._robo_radius * np.cos(pos[2:]), -1.2 * self._robo_radius * np.sin(pos[2:])]).reshape(2, 1)
        pygame.draw.line(self.screen, self._robo_dirc_color, (start + start_change)[:, 0].astype(float),
                         (start + end_change)[:, 0].astype(int), self._field_line_width)
        id_indicator_pos = (pos[:2] * self.scale - [0.5 * self._robo_radius, 0.75 * self._robo_radius]).astype(float)
        self.screen.blit(self.font.render(str(player_id), True, (0, 0, 0)), id_indicator_pos)

    def accrue_data_from_simulator(self, team_1=None, team_2=None, ball=None) -> (tuple, tuple, tuple):
        """
        Accrue data from game simulation - players and ball positions, # of goals, game time etc
        :return:
        """
        if not self._data_provided:
            return self._temp_generate_random_data_from_simulation()
        else:
            self._data_provided = False
            return self._data_provided_buffer

    def send_game_state(self, team_1, team_2, ball, score):
        self._data_provided = True
        self._data_provided_buffer = (team_1, team_2, ball, score)
