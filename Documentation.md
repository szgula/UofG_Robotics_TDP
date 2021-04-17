## Plan according to the report guidlines

1. Abstract – Brief summary of the report. Usually only half a page for a report of this type.
2. Introduction – Outline of the project requirements in your own words.
3. Methodology – Description of the implementation aspects for each stage of the
   assignment e.g. creation of model, validation, control system design, tests.
4. Results and Analysis – Required results to illustrate completion of each stage and
   discussion of the analysis of these results.
5. Team Performance Analysis – Evaluation of the performance of your team, including a
   comparison of your estimated and actual time/costs.
6. Conclusions – Findings from the methodology implementation and analysis of the
   results. Overall assessment of the project outcomes.
7. References – List of documents used in this assignment.
8. Appendices – Additional material not needed in the main sections of the report e.g.
   Matlab code, full Simulink block diagrams.

**!! The report should indicate which team members were responsible for authoring the separate sections of the report e.g. Section 1 (written by Team Member 1 and Team Member 2).**

**!! All figures, tables and equations must be numbered in an appropriate manner. Figures and tables need to have a descriptive caption. Axes on graphs need to be labelled appropriately.**

# Table of Contents

1. [Introduction](#introduction)
   - [Problem Statement](#problemstatement)
   - [Objective](#objective)
   - [Expectations](#excpectations)
2. [Methodology](#methodology)
   - [Architecture + ROS](docs/Ros_Methodology/Architecture.md)
   - [Simulation](docs/Ros_Methodology/Simulation.md)
   - [Visualisation](docs/Ros_Methodology/Visualisation.md)
   - Strategy
     - [Team 0](docs/Strategy/Strategy_Team_0.md)
     - [Team 1](docs/Strategy/Strategy_Team_1.md)
   - Basic Actions
     - [Go To Point (empty - Shrey)](docs/Basic_Actions/Go_To_Point.md)
     - [Go Around Point (empty - Shrey)](docs/Basic_Actions/Go_Around_Point.md)
     - [Kick Ball (empty - Shrey)](docs/Basic_Actions/Kick_Ball.md)
     - [Rotate towards (empty - Shrey)](docs/Basic_Actions/Rotate_Towards.md)
   - Strategic Actions
     - [Redirect Ball](docs/Strategic_Actions/Redirect_Ball.md)
     - [Pass Ball](docs/Strategic_Actions/Pass_Ball.md)
     - [Score Goal](docs/Strategic_Actions/Score_Goal.md)
     - [Avoid Obstacles](docs/Strategic_Actions/Avoid_Obstacle.md)
     - [Dribble](docs/Strategic_Actions/Dribble.md)
     - [Cover Opponent](docs/Strategic_Actions/Cover_Opponent.md)
     - [Go To Strategic point](docs/Strategic_Actions/Go_To_Strategic_point.md)
   - Supporting functions
     - Kicking the ball
       - [Check if pass the ball is feasible](docs/Supporting_functions/Kick_Ball/Ball_Pass.md)
       - [Find player to Pass ](docs/Supporting_functions/Kick_Ball/Player_Pass.md)
       - [Check if score the goal is feasible](docs/Supporting_functions/Kick_Ball/Goal_Pass.md)
     - Capture the ball
       - [Get the soonest ball collision](docs/Supporting_functions/Capture_Ball/Soonest_Ball.md)
       - [Get capture position (ball pos at time)](docs/Supporting_functions/Capture_Ball/Capture_Position.md)
       - [Get the soonest contact](docs/Supporting_functions/Capture_Ball/Soonest_Contact.md)
3. [Results and Analysis (Feng & Ivan) (empty)](docs/Result_Analysis.md)
   - Experiments
   - Test cases
   - etc
4. [Project organisation](docs/Organisation/Project_Organisation.md)
   - [Team Performance Analysis (empty)](docs/Organisation/Team_Performance.md)
5. [Conclusions (Omar)](docs/Conclusions.md)
6. [References (empty)](docs/References.md)

# Introduction <a name="introduction"></a>

This project presents an engineering pipeline of allowing robotic components to successfully complete a football match based on behavioural controllers and impulsive strategy to solve the international RoboCup challenge. 
The simulation involves two teams containing five robots, competing against each other for the win. 
The robots were designed to be differential drive machines relying on the input voltages to perform their assigned actions.
This solution is demonstrated in a simulated environment taking into account all the rules and procedures demanded by the challenge. 
The main building blocks of this project are as follows:
1. Robot Operating System (ROS): This platform combines all the project's components into one distributed system that treats each executable program as a thread in said system.
2. Gazebo: Software to present the simulation in an easy and straightforward manner. This program encapsulates all the different components and presents them in a structured simulation mimicking the environment. 
3. Python: Programming language used to develop the executable programs representing the robot's decision making, planning and actions.
4. PyGame: Visualisation tool to plot the main components of the environment: football field (e.g. goals, line separators), robot structures (chassis size, scale, orientation and form).

All of these tools allowed us to work as a team and structure our solution as fast-paced sprints alongside long-achieving milestones.
   
## Problem Statement <a name="problemstatement"></a>

This challenge proposes the implementation of a simulation of behavioural robots managing to rely on their built-in algorithms and challenging each other in a football match environment. 
This problem is divided into several sub-problems to be tackled:
1. System Architecture: Finding the optimal system design to represent and structure our execution flow.
2. Simulation + Visualization: Implementing an efficient representation of the problem through simulating the physical elements of the environment and illustrating its different components appropriately.
3. Strategy and Decision Making: Implementing from scratch the theory behind the strategy and decision making on a team-wide level.


## Objective <a name="objective"></a>
The main objective is to generate appropriate solutions for the sub-problems stated above. Dividing the project into sub-modules allows us to interpret the solution as a bottom-up approach and facilitates assigning tasks to each team member. As stated before, the project outline is divided into short-term sprints and long-term milestones. Hence, each sub-module will be guaranteed a milestone and each sprint will dedicate tasks to achieve a specific milestone.

### Expectations <a name = "expectations"></a>

The teams are expected to withhold a full football match with all rules followed and actions supported with behavioural reactions and impulsive response in the environment.
- Team 0 is expected to be the test case. The latter means that the players will perform minimal effort, enough to cooperate and compete, but not optimal as a perfect and flawless team. 
- Team 1 is expected to have superior decision making and strategy, and to have the advantage in both scoring and possession in the football match. 

Hence, our main expectation is that Team 1 will win the match and overcome all Team 0's strategies.


### Level of Abstraction

The problem statement gives us the freedom to choose the level of abstraction of the simulation.
The level of abstraction here means the lowest level of detail of physics, which the simulation
is going to take under consideration.
We have chosen that we can impart instantaneous angular velocity to
the wheels. This will make the interface of further development of functions easier for all team members.

### Choice of ROS and Python as the Technology

We have chosen ROS because...

We have chosen Python as it is slowly becoming the language of choice for scientific computing.
Python is also a general-purpose language which means any practice of it comes under the category
of "Transferable learning", i.e. the knowledge of the programming language carries over to application
in other budding fields such as A.I. and Data Science. For scientific computing and linear algebra
Python has a state of the art library called NumPy. Numpy has data structures, indexing and
syntax almost identical to Matlab, which makes it easy for people who are well versed in speaking Matlab
to migrate to Pythondom.

Therefore we must develop a playing environment, the laws of physics of the simulated world, with models of the ball
and differential drive robots the behavioural algorithms of such robots. We shall do this by implementing ROS and the
Python programming language.

## Overview

Here a bird’s eye of the Project and upcoming chapters should be given.


