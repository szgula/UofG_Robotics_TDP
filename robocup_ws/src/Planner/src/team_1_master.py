import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from team_master import TeamMasterServer, TeamMaster
from goalkeeper_controller import Team1GoalkeeperController
from striker_controller import Team1StrikerController, RotateController
from stay_in_place_controller import NullController
from game_interfaces.msg import Position, PlayerCommand
from BasicCommonActions.go_to_action import go_to_fast, receive_and_pass_action
from defence_controller import Team1DefenceController
import numpy as np
import logging
from BasicCommonActions.plan_supporting_functions import TeamMasterSupporting


class TeamMaster1(TeamMaster):
    def __init__(self):
        team_id = 1
        super().__init__(team_id)
        self.goalkeeper_logic = Team1GoalkeeperController(1)
        self.striker_left_logic = NullController()
        self.striker_right_logic = NullController()
        self.defence_left_logic = Team1DefenceController(2)
        self.defence_right_logic = Team1DefenceController(-2)
        self.players_logic_was_updated = True

    def plan(self):
        super(TeamMaster1, self).plan()
        self.get_local_actions()

    def get_local_actions(self):
        out_tms = TeamMasterSupporting.get_soonest_contact(self.team_position, self.opponents_position)
        team_can_get_to_ball, players_capture_time, opponent_can_get_to_ball, opponents_capture_time = out_tms
        player_id_min_time = int(np.argmin(players_capture_time))
        opponent_id_min_time = int(np.argmin(opponents_capture_time))

        if team_can_get_to_ball and (player_id_min_time == self.striker_right_idx or
                                     player_id_min_time == self.striker_left_idx):
            capture_pos = TeamMasterSupporting.get_ball_pos_at_time(players_capture_time[player_id_min_time],
                                                                    self.team_position.ball_pos_efcs,
                                                                    self.team_position.ball_vel_efcs)

            self.actions[player_id_min_time] = self.simple_go_to_point_and_kick(self.team_position.players_positions_efcs[player_id_min_time],
                                                                                capture_pos, self.team_position.ball_pos_efcs)
        if team_can_get_to_ball and (player_id_min_time == self.defence_right_idx or
                                     player_id_min_time == self.defence_left_idx):
            capture_pos = TeamMasterSupporting.get_ball_pos_at_time(players_capture_time[player_id_min_time],
                                                                    self.team_position.ball_pos_efcs,
                                                                    self.team_position.ball_vel_efcs)
            self.actions[player_id_min_time] = self.simple_go_to_point_and_receive(self.team_position.players_positions_efcs[player_id_min_time],
                                                                                   capture_pos, self.team_position.ball_pos_efcs)


        # TODO: move other striker to line of opponent defence
        # TODO: receive the ball, rotate towards opponent's net or other striker and kick/pass

    def simple_go_to_point_and_kick(self, robot_state: Position, target: Position, ball_position: Position):
        d = np.hypot(robot_state.x - ball_position.x, robot_state.y - ball_position.y)
        action = 0
        if d < 0.15:
            action = 1
        return PlayerCommand(*go_to_fast(robot_state, target), action)

    def simple_go_to_point_and_receive(self, robot_state: Position, target: Position, ball_position: Position):
        d = np.hypot(robot_state.x - ball_position.x, robot_state.y - ball_position.y)
        action = 0
        if d < 0.15:
            _, player_to_pass = TeamMasterSupporting.find_safe_players_to_pass(self.team_position, self.opponents_position, self.team_id)
            temp_target_pos = self.team_position.players_positions_efcs[player_to_pass]
            raw_command = receive_and_pass_action(robot_state, temp_target_pos, ball_position)
            command = PlayerCommand(*raw_command)
        else:
            command = PlayerCommand(*go_to_fast(robot_state, target), action)
        return command


if __name__ == "__main__":
    team_mater_1 = TeamMaster1()
    TMS = TeamMasterServer(team_mater_1)
