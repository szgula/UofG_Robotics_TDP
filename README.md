# UofG_Robotics_TDP
Robotics Team Design Project at University of Glasgow



## Run the game master and game simulator

1. source the base ROS
2. catkin_make the workspace
3. source workspace build
4. using ros launch:
    - go to workspace dir
    -  ```roslaunch GameEngine game.launch```
5. Or using ros run:
    - go to workspace dir
    - ```rosmaster```
    - ```rosrun GameEngine game_master.py```
    - ```rosrun GameEngine game_simulator.py```
    

## Configure the PyCharm development environment

### Pre-requirements 
1. Install PyCharm
2. Create an alias to Pycharm 
3. Create an alias to main ROS source

### Setup
1. Source base ROS
2. from same terminal launch PyCharm
3. Create a virtual environemtn (call it: ```.venv```)
4. Create launch configuration
    - change the launch directory to GameEngine
    

...to be continue