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
     - [Go To Point](docs/Basic_Actions/Go_To_Point.md)
     - [Go Around Point](docs/Basic_Actions/Go_Around_Point.md)
     - [Kick Ball](docs/Basic_Actions/Kick_Ball.md)
     - [Rotate towards](docs/Basic_Actions/Rotate_Towards.md)
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
3. [Results and Analysis](docs/Result_Analysis.md)
   - Experiments
   - Test cases
   - etc
4. [Project organisation](docs/Organisation/Project_Organisation.md)
   - [Team Performance Analysis (empty)](docs/Organisation/Team_Performance.md)
5. [Conclusions (Omar)](docs/Conclusions.md)
6. [References (empty)](docs/References.md)

# Introduction <a name="introduction"></a>

This project is about designing and simulating the RoboCup challenge - the football match with robots as players. 
The simulation involves two teams of five robots competing against each other. 
The robots are designed as two-wheel differential drive machines with an internal logic processor and a capability to communicate with different robots. 
Our design is presented in a simulated environment that takes into account all constraints of the challenge. 
The main building blocks of this project are the following:
1.	Robot Operating System (ROS) - a platform that combines all the design's components into one distributed system that treats each executable program as a thread in the given system.
2.	Python - a programming language used to develop the executable programs representing the robot's decision-making process, planning and actions.
3.	PyGame - a visualisation framework to display the main components of the environment: football field (e.g., goals, line separators), robot structures (chassis size, scale, orientation and form).

All of these tools allowed us to work as a team and structure our solution as fast-paced sprints alongside long-term milestones.

## Problem Statement <a name="problemstatement"></a>

The project focuses on designing with a team of collaborative robots that can compete with opponents in a simulated game. This problem was split into several sub-problems to be tackled one by one:
1.	System Architecture - finding an optimal system design to represent and structure our execution flow.
2.	Simulation + Visualization - implementing an efficient representation of the problem through simulation of the physical elements of the environment and visualising its different components appropriately.
3.	Strategy and Decision Making - implementing from scratch the theory behind the strategy and decision-making on a team level.


## Objective <a name="objective"></a>
The main objective is to generate appropriate solutions for the sub-problems stated above. Dividing the project into sub-modules allows us to interpret the solution as a bottom-up approach and facilitates assigning tasks to each team member. As stated earlier, the project outline is split into short-term sprints and long-term milestones. Hence, each sub-module is linked with a milestone and each sprint focuses on achieving a specific milestone.


### Expectations <a name = "expectations"></a>

The robot teams are expected to complete a full football match with all rules followed and actions supported with behavioural reactions and impulsive response in the environment.
   - Team 0 is expected to be a semi-smart team. It means that the players will make a minimal effort - enough to cooperate and compete, but not as effective as a high-performing team.
   - Team 1 is expected to have superior decision-making and strategy, and to have the advantage in both scoring and ball possession in a football match.

Hence, our main expectation is that Team 1 will win each match and overperform all Team 0's strategies.


### Choice of Technology

We have chosen Python as it is the most popular scripting language with great community support and a number of open-source libraries. Moreover, Python is a general-purpose language, which means any practice of it comes under the category of "Transferable learning" (i.e., the knowledge of the programming language carries over to application in other growing fields such as A.I. and Data Science).

In addition to Python, we have decided to use Robot Operating System (ROS) as a backend infrastructure for the project. ROS is the fastest-growing framework for robotics application with a great open community and many available packages. Specific reasons why we selected ROS are further discussed in the Architecture section.


