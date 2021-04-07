## Plan according to the report guidlines 
1) Abstract – Brief summary of the report. Usually only half a page for a report of this type.
2) Introduction – Outline of the project requirements in your own words.
3) Methodology – Description of the implementation aspects for each stage of the
assignment e.g. creation of model, validation, control system design, tests.
4) Results and Analysis – Required results to illustrate completion of each stage and
discussion of the analysis of these results.
5) Team Performance Analysis – Evaluation of the performance of your team, including a
comparison of your estimated and actual time/costs.
6) Conclusions – Findings from the methodology implementation and analysis of the
results. Overall assessment of the project outcomes.
7) References – List of documents used in this assignment.
8) Appendices – Additional material not needed in the main sections of the report e.g.
Matlab code, full Simulink block diagrams.

__!! The report should indicate which team members were responsible for authoring the separate sections of the report e.g. Section 1 (written by Team Member 1 and Team Member 2).__

__!! All figures, tables and equations must be numbered in an appropriate manner. Figures and tables need to have a descriptive caption. Axes on graphs need to be labelled appropriately.__

# Table of Contents
1. [Introduction - Omar](#introduction)
   - [Problem Statement](#problem statement)
   - [Objective](#objective)
2. [Methodology](#methodology)
   * [Architecture + ROS (empty - Szymon & Omar)](docs/Ros_Methodology/Architecture.md)
   * [Simulation](docs/Ros_Methodology/Simulation.md)
   * [Visualisation (empty - Feng)](docs/Ros_Methodology/Visualisation.md)
   * Strategy
      * [Team 0](docs/Strategy/Strategy_Team_0.md)
      * [Team 1](docs/Strategy/Strategy_Team_1.md)
   * Basic Actions   
      * [Go To Point (empty - Shrey)](docs/Basic_Actions/Go_To_Point.md)
      * [Go Around Point (empty - Shrey)](docs/Basic_Actions/Go_Around_Point.md)
      * [Kick Ball (empty - Shrey)](docs/Basic_Actions/Kick_Ball.md)
   * Strategic Actions
      * [Pass Ball (empty - Szymon)](docs/Strategic_Actions/Pass_Ball.md)
      * [Score Goal (empty - Ivan)](docs/Strategic_Actions/Score_Goal.md)  
      * [Avoid Obstacles](docs/Strategic_Actions/Avoid_Obstacle.md)
      * [Dribble](docs/Strategic_Actions/Dribble.md)
      * [Cover Opponent (empty - Ivan)](docs/Strategic_Actions/Cover_Opponent.md)
      * [Go To Strategic point (Empty - Omar)](docs/Strategic_Actions/Go_To_Strategic_point.md)
   * Supporting functions
      * Kicking the ball
         * [Check if pass the ball is feasible (Feng)](docs/Supporting_functions/Kick_Ball/Ball_Pass.md)
         * [Find player to Pass  (Omar)](docs/Supporting_functions/Kick_Ball/Player_Pass.md)
         * [Check if score the goal is feasible (Ivan)](docs/Supporting_functions/Kick_Ball/Goal_Pass.md)
      * Capture the ball   
         * [Get the soonest ball collision (Szymon)](docs/Supporting_functions/Capture_Ball/Soonest_Ball.md)
         * [Get capture position (ball pos at time) (Szymon)](docs/Supporting_functions/Capture_Ball/Capture_Position.md)
         * [Get the soonest contact (Szymon)](docs/Supporting_functions/Capture_Ball/Soonest_Contact.md)
3. [Results and Analysis (Feng & Ivan) (empty)](docs/Result_Analysis.md)
   * Experiments
   * Test cases
   * etc
4. [Team Performance Analysis (empty)](docs/Team_Performance.md)
5. [Conclusions (Omar) (empty)](docs/Conclusions.md)
6. [References (empty)](docs/References.md/)

Introduction <a name="introduction"></a>
============

This project presents an engineering pipeline of allowing robotic components to sucessfully complete a football match based on behavioral controllers and impulsive strategy to solve the international RoboCup challenge. <br>The simulation involves two teams containing five robots, competing against each other for the win. <br>The robots were designed to be differential drive machines relying on the input voltages to perform their assigned actions.<br>This solution is demonstrated in a simulated environment taking into account all the rules and procedures demanded by the challenge. <br> The main building blocks of this project are as follows:<br> 1- Robot Operating System (ROS): This platform combines all the project's components into one distributed system that treats each executable program as a thread in said system.<br> 2- Gazebo: Software to present the simulation in an easy and straighforward manner. This program encapsulates all the different components and presents them in a structured simulation mimicing the environment.<br> 3- Python: Programming language used to develop the executable programs representing the robot's decision making, planning and actions.<br> 4- Pygame: Visualisation tool to plot the main components of the environment: football field (e.g. goals, line separators), robot structures (chassis size, scale, orientation and form).<br>All of these tools allowed us to work as a team and structure our solution as fast-paced sprints alongside long-achieving milestones.

### Problem Statement 

This challenge proposes the implementation of a simulation of behavioral robots managing to rely on their built-in algorithms and challenging each other in a football match environment. This problem is divided into several sub-problems to be tackled:<br>1- System Architecture: Finding the optimal system design to represent and structure our execution flow.<br>2- Simulation + Visualization: Implementing an efficient representation of the problem through simulating the physical elements of the environment and illustrating its different components appropriately.<br>3- Strategy and Decision Making: Implementing from scratch the theory behind strategy and decision making on a team-wide level.


### Objective <a name="methodology"></a>
The main objective is to generate appropriate solutions for the sub-problems stated above. <br>Dividing the project into sub-modules allows us to interpret the solution as a bottom-up approach and facilitates assigning tasks to each team member.<br>As stated before, the project outline is divided into short-term sprints and long-term milestones. Hence, each sub-module will be guaranteed a milestone and each sprint will dedicate tasks to achieve said milestone. 

### Level of Abstraction
The problem statement gives us the freedom to choose the level of abstraction of the simulation.
The level of abstraction here means the lowest level of detail of physics, the simulation
is going to take under consideration.
We have chosen that we can impart instantaneous angular velocity to 
the wheels. This will make the interface of further development of functions easier for all team members.

### Choice of ROS and Python as the Technology
We have chosen ROS because

We have chosen Python as it is slowly becoming the language of choice for scientific computing.
Python is also a general purpose language which means, any practice of it comes under the category
of "Transferable learning", i.e. the knowledge of the programming language carries over to application
in other budding fields such as A.I. and Data Science. For scientific computing and linear algebra
Python has a state of the art library called NumPy. Numpy has data structures, indexing and 
syntax almost identical to Matlab, which makes it easy for people who are well versed in speaking Matlab
to migrate to Pythondom.

Therefore we must develop a playing environment, the laws of physics of the simulated world, with models of the ball
and differential drive robots the behavioural algorithms of such robots. We shall do this implementing ROS and the
Python programming language.


Overview
--------

Here a bird’s eye of the Project and upcoming chapters should be given.

ROS nodes
=========

Since ROS implementation gives the structure to the project at the
highest level it must be discussed first. Brief description of what is
ROS. How it has been implemented.

Basic Queries
=============

For calculating the next action, the players need to know the position of
the ball and all other players on the field. For this the players can
query into the state of the simulator using various functions. Such
functions are very important in the implementation of the decision tree
which is discussed in the next chapter.


### Has ball or not
This function returns a boolean value of true if a ball is in a close proximity(defined by a proximity threshold) 
of the player. It is then assumed that the player posseses the ball and will take actions which are appropriate.

### Check if the ball is free
This function returns a value of True when the ball is outside the
proximity zone of all the players on the field.

### Check if the ball is in the teams half
This returns true if the ball is in the first half of the field. This puts the team in defence mode in the decision tree.

### Check if goal can be scored or not
This function checks if a player who has a ball has a clear line of sight to the goal of the opposite team. It returns true
if such is the case.

### Check if dribble is safe or not
This function returns a boolean value of true if none of the opponents are present in an imaginary square around the
ball.


### Check for pass

### Check if the ball can be passed by a defender to another

### Obstacle distance calculation


### Check which player can reach fastest to the ball
This function returns the player id which can intercept a moving ball in the minimum possible time and also the position
that the player needs to go to.

### Check which striker of the team is closest to the ball
This function returns the player id of the striker which is closest to the ball.

Basic Actions
=============

Atomic actions such as going to a point, scoring the goal, passing the
ball, collision avoidance etc. More complex actions such as pass and receive actions.

### Basic Actions   
  * [Go To Point (empty)]()
  * [Go Around Point (empty)]()
  * [Kick Ball (empty)]()
### Strategic Actions
  * [Pass Ball (empty)]()
  * [Score Goal (empty)]()  
  * [Avoid Obstacles](docs/Actions_Avoid_Obstacle.md)
  * [Dribble](docs/Actions_Dribble_Ball.md)
  * [Cover Opponent (empty)]()
  * [Go To Strategic point (?? Empty - is it different than go to point)]()



Output
======

Appreciation of the output.

Future Scope of Improvisation
=============================

Maybe AI can be implemented. Like reinforced learning.

Conclusion
==========

What did we learn.
