# Methodology
This section of the documentation is about the project's structure, architecture and implementation aspects.
We first present the project division and architecture. Next, we go into details about high-level task definition and algorithm design. 
Finally, the implementation details about low-level functions are presented.

# ROS and Project Architecture
##### [Go back to main page](../../Documentation.md)

## ROS

Robot Operating System (ROS) is middleware software that provides a framework to create real-time and concurrent programs.
Thanks to the mentioned properties, it is widely used in robotics projects, including both, hardware and simulation-based projects.

In the early stage of the project, we have decided to split the project into multiple modules. 
Such an approach guarantee clear areas of responsibilities of each module, guarantees scalability and provide a relatively simple interface to replace modules with newly created or external software. 
Besides, it simplified team development, as it was easy to manage code dependencies and code merge conflicts.

The greatest overhead comparing to procedural-sequential architecture (the one we also considered) was the big responsibility of communication interfaces.
The ROS based architecture requires clearly defined interfaces between modules and after it is done, there is little flexibility to change it.
Fortunately, thanks to the well-defined project requirements, the interface definition becomes a relatively simple task.

## Project Architecture

The project was divided into four nodes:
1. ___Game Master___ is the game wrapper responsible for game initialisation, checking game rules, counting the goals and keep track of the game time. 
   Last but not least, this node is a proxy between teams nodes and the simulator.
2. ___Simulator___ - is a physics simulation engine that replaces the real world and robots. It takes all robots actions and provides a discrete-time update of the whole environment. 
3. ___Team 0___ - is a team 0 logic layer that outputs the actions for specific robots.
4. ___Team 1___ - a similar node to the previous, but for team 1.


For the communication between the nodes, we selected the ROS services - the client-server communication paradigm implementation. 
This enables the project to have a synchronized behaviour of all components. 
One can point it provide a limitation to reuse the same architecture in real robots, but it can be overcome by adding stream communication between robots and team shared logic.

Mentioned architecture with information flow between nodes is presented in Figure 1. 

![Diff drive](../Figures/ROS_architecture.png)
__Figure 1:__ Project architecture based on the ROS inheritance: including four nodes connected with services 
