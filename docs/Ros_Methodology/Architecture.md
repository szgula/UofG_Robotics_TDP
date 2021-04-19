# Methodology
In this section, we will discuss the project's structure, architecture and implementation aspects.
In the first part, we present project division and architecture, followed by a detailed description of high-level task definition and algorithm design. Next, we discuss implementation details of low-level functions are presented.

# ROS and Project Architecture
##### [Go back to main page](../../Documentation.md)

## ROS

Robot Operating System (ROS) is middleware software that provides a framework to create real-time and concurrent programs.
Thanks to the mentioned properties, it is widely used in robotics projects, including both hardware and simulation-based projects.

In the early stage of the project, we decided to split the project into multiple modules. 
It allowed us to clearly define areas of responsibilities of each software module. 
Such an approach guarantees scalability and provides a relatively simple interface to replace modules with newly created or external software. 
Moreover, it simplifies team development, as it is easier to manage code dependencies and code merge conflicts.

In comparison to the other considered architecture (procedural-sequential architecture), the greatest overhead of our approach was the high pressure on well-defined communication interfaces. The ROS based architecture requires clearly defined interfaces to communicate between modules and after it is done, there is little flexibility to change it.
Fortunately, thanks to the well-defined project requirements, the interface definition becomes a relatively simple task.

## Project Architecture

The project was divided into four nodes:
1. ___Game Master___ - is a game wrapper responsible for game initialisation, checking game rules, counting the goals and keeping track of game time. 
   Last but not least, this node is a proxy between teams nodes and the simulator.
2. ___Simulator___ - is a physics simulation engine that mimics the real world and robots. It takes all robots actions and provides a discrete-time update of the whole environment. 
3. ___Team 0___ - is a Team 0 logic layer that outputs actions for specific robots.
4. ___Team 1___ - a similar node to the previous one, but for Team 1.


For the communication between the nodes, we selected the ROS services - the client-server communication paradigm implementation. 
This enables the project to have a synchronized behaviour of all components. One limitation of this architecture is that it may be hard to reuse it for real robots. However, this limitation can be overcome by adding stream communication between robots and team shared logic.


The considered architecture with information flow between nodes is presented in Figure 1. 

![Diff drive](../Figures/ROS_architecture.png)
__Figure 1:__ Project architecture based on the ROS inheritance: including four nodes connected with services 
