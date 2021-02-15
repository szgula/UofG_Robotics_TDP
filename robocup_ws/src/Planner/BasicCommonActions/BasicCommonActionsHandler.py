from BasicCommonActions.Action import Action


class BasicCommonActionsHandler:

    def __init__(self, action: Action):
        self._action = action

    @property
    def action(self) -> Action:
        return self._action

    @action.setter
    def action(self, action: Action) -> None:
        self._action = action

    def handle(self):
        return self._action.do_algorithm()
