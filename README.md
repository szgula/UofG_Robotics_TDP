# UofG_Robotics_TDP
Robotics Team Design Project at University of Glasgow.  
## Robocup ROS Workspace Basics:  
1. Head to robocup_ws (workspace):   
`cd robocup_ws`  
2. Build dependencies and packages:  
`catkin_make`  
3. Source the project so that ROS can locate all packages:  
`source devel/setup.bash`  
4. Once in **/src**, run:  
`roslaunch robocup_gazebo robocup_world.launch`  
