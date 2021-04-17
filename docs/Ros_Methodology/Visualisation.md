# Visualization

##### [Go back to main page](../../Documentation.md)

# Architecture

The game visualiser is embedded in the `GameSimulationServer` as a utility class. `visualizer.py` contains all the functional code for visualisation and there is only one relevant class to call this function when necessary.

Such architecture brings all logic about visualization together and makes the simulator independent from the display functionalities. It keeps coupling between modules low and ensures code transparency.

## Workflow

Once the game starts, the game master will start a loop with the length dependent on the value of `full_game_length`.

In each iteration, the master will not only request `team0_server` and `team1_server` for the latest command for every player but also send a request to `simulator_server` to update all the visualisation display.

```
rospy.Service(r'game_engine/game_simulation', SimulationUpdate, self.handle_simulation_call)
```

![Visualizer workflow](../Figures/visualization_workflow.png)  
**Figure 1:** Visualiser Workflow

## Display components

Visualiser is responsible for nine components' displaying, including:

1. **id indicator**: shows an id number of every player
2. **direction indicator**: shows the direction every player points to and changes in real-time.
3. **offside warning line**: displays the possible offside line for a warning purpose

![Main Components](../Figures/Visualization_main_components_of_field.png)
**Figure 2:** Components for visualisation

![Main Components](../Figures/Visualization_main_component_on_field_map.png)
**Figure 3:** Components for visualisation marked on a map

## Available properties for setting display style

We can use the following properties to set the style for every component.

```
self._robo_radius = 10
self._ball_radius = 5
self._field_line_color = (255, 255, 255)
self._offside_line_color = (255, 105, 180, 100)
self._field_color = (55, 170, 80, 0)
self._robo_dirc_color = (255, 0, 0)
self._field_line_width = 3
self._center_circle_radius = 1 * display_scale
self._goal_area_width = 1.5 * display_scale
self._goal_area_height = 3 * display_scale
self._penalty_arc_area_radius = 0.8 * display_scale
self._penalty_arc_center_distance = 1.1 * display_scale
self._margin = 0.3 * display_scale
self._gate_height = 2 * display_scale
self._gate_color = (170, 170, 170)
```
