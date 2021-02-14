from BasicCommonActions.Action import Action
from robot_model import RobotModel


class DoNothingAction(Action):

    def do_algorithm(self):
        if isinstance(self._data[0], RobotModel):
            return self._data[0].get_velocity_components_wcs()
        else:
            return None
