<h2 align="center">UofG_Robotics_TDP</h2>  
<p align="center">
  <a href="https://github.com/szgula/UofG_Robotics_TDP">
    <img src="https://github.com/Yifan-Xie/Image/blob/Robotic-TDP/pictures/Robotic_TDP.png" alt="Logo" >
  </a>
  
  <p align="center">
    Robotics Team Design Project at University of Glasgow
    <br />
    <br />
</div>

<!-- TOC -->
<details open="open">
  <summary><h2 style="display: inline-block">Contents</h2></summary>
  <ol>
    <li><a href="#Robocup ROS Workspace Basics">Robocup ROS Workspace Basics</a></li>
    <li><a href="#Run the game master and game simulator">Run the game master and game simulator</a></li>
    <li><a href="#Configure the PyCharm development environment">Configure the PyCharm development environment</a></li>   
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#Packages">Packages</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

## Robocup ROS Workspace Basics
1. Head to robocup_ws (workspace):   
`cd robocup_ws`  
2. Build dependencies and packages:  
`catkin_make`  
3. Source the project so that ROS can locate all packages:  
`source devel/setup.bash`  
4. Once in **/src**, run:  
`roslaunch robocup_gazebo robocup_world.launch`  
<div align="center"> 
<br />  
</div>

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
    
<br />

## Configure the PyCharm development environment

### Add alias 
1.For ROS
2. For PyCharm

### PyCharm - setting the project
1. In Pycharm open project with UofG_Robotics_TDP as a source folder
2. ``file -> Settings -> Project: UofG_Robotics_TDP -> Project Structure``
3. Right click on ```robotcap_ws``` folder and click on ```source```

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

1) git clone
2) git checkout speccific branch
3) source base ROS ```source /opt/ros/__vrsion__/setup.bash```
4) cd robotcup
5) catkin_make
6) source devel/setup.bash
7) cd ../
8) sudo pip3 install -r requirements.txt (if pip3 is not working install it)
9) cd robotcup
10) roslaunch GameEngine game.launch


### How to test:
Simple ros launch:

```roslaunch GameEngine game.launch```

ros launch with the ball init position at x = 1, y = 2 in world coordinate system 

```roslaunch GameEngine game.launch simulator_args:="--ball_pos 1 2"```

ros launch with the ball init position at x = 1, y = 2 (in world coordinate system) and vel x = 0 y = 0.4

```roslaunch GameEngine game.launch simulator_args:="--ball_pos 0.5 0.5 --ball_vel 0 0.4"```

ros launch with team 1 at new positions: player1_x, player1_y, player2_x, player2_y, ... , player5_y

```roslaunch GameEngine game.launch simulator_args:="--team_1_init_pos -2 2 -2 -2 2 1 2 -1 -4 0"```


## Roadmap
The step to finish the project.

<!-- Getting Started -->
## Packages
In ```requirement.txt```

<!-- Usage -->
## Usage
The method to use it.

<!-- Contributing -->
## Contributing
How to contribute the project.

<!-- License -->
## License
Distributed under the MIT License. See `LICENSE` for more information.

<!-- Contact Info -->
## Contact
Szymon  @[Szymon](https://github.com/szgula)
<br />
<br />
Feng  @[Feng](https://github.com/fengfengFinn)
<br />
<br />
Omar  @[Omar](https://github.com/OmarJabri7)
<br />
<br />
Yifan  @[Yifan](https://github.com/Yifan-Xie)
<br />
<br />
Shrey @[Shrey](https://github.com/shreyExp)
<br />
<br />
Project address @[Project address](https://github.com/szgula/UofG_Robotics_TDP)
