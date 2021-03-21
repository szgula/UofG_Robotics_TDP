Introduction
============

This project is about simulating a football game played by robots.
Such projects give us a way to test out the theory we have learnt, develop an understanding
of how to design and implement projects of a substantial size and to co-ordinate in a team.
We can hope to learn various aspects of software engineering, logic, algorithms, mathematics
and kinematics. Therefore this project becomes more than just a game, it is an opportunity to
learn at a fast pace.

Problem Statement
-----------------

In a team we must create a simulation of a robot soccer team and the playing environment. 
We must develop a playing environment, behavioural algorithms and a visualisation of the
pitch and players.
There must be two strikers, two defenders and a goalkeeper.
We must consider either two, three of four wheeled robots.

Note: Although the platform of choice was Matlab(with it's convenient packages
such as Simulink and Stateflow), we have chosen Python as the technology to go ahead with for reasons
staded in the next section.  


Objective
---------
The objective mirrors the problem statement. There are a few points to consider.

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

Physics
=======

In this section the coordinate system of the field and the physics of
the simulation is discussed.

Coordinate System
-----------------

The size of the field is 10 units in horizontal direction and six units in the vertical
direction. The origin lies at the centre of the field which makes x(horizontal direction) lie between -5 and 5 and
y(vertical direction) lie between -3 and 3.

Kinematics
----------

### Step

The position of Robots and the ball are kept in a buffer and updated together in
synchronization with each step in the simulation. All robots and the ball
have a common clock. The new position of a robot is determined by its instantaneous angular speed of the wheels, the delta t and
the radius of the wheels. New position of the ball is determine by its instantaneous speed and a coefficient of friction.

### Ball Model
The ball is considered a point object which is always decelerating due to a coefficient of friction. The ball
will bounce according to the laws of elastic collision with round players and the walls of the field as discussed
in the following sections.

#### Mass
Mass is defined in the model of the ball as 0.1 units.

#### Position
The ball class has a variable position (defined separately for x and y coordinates) 
which gets updated at each step according to the formula:

#### Velocity
The velocity of the ball is defined for both x and y coordinates and is dependent on the friction (negative acceleration)
and time elapsed according to the formula:

#### Friction
The coefficient of dynamic friction is taken to be 0.01. This gives negative acceleration to the ball at each step
until the next event happens and a new velocity is imparted to the ball.

#### Collision
Collision of the ball with a wall or with a player is an important event. Collision of a ball is identical to
law of reflection of a light ray on a mirror, whether it is collision with a wall (flat surface) or a robot (round surface).
Such collisions happen without any loss in the magnitude in the velocity of the ball. It is just the direction which
changes. The incident angle equals the reflection angle eventually.

#### Players Action
During the play. The players can impart certain changes to ball's trajectory. 
This in the balls code is called player's actions. They are kick and receive as explained below.

##### Kick
In this event the ball is imparted a velocity which is along the direction of the vector joining the ball and the centre point of
the player. The imparted velocity's magnitude is fixed. The kick is only executed only when the ball is within a certain
distance threshold of the position of the player.

##### Receive
In the receive action, the ball is imparted the velocity of the player which receives the ball if the ball
is within a certain distance threshold.

### Robot Model

The robots are implemented using two wheel differential drive kinematics because they are also easier to implement physically.
In the differential drive model each wheel is imparted with an independent angular velocity. The speed, the heading
angle and the next position of the robot are derived out of the angular velocities of the wheels and the wheel's radius.

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

### Check which striker of the team is closest

Basic Actions
=============

Atomic actions such as going to a point, scoring the goal, passing the
ball, collision avoidance etc.

### Go to point

### Go around a point

### Rotate towards a direction

### Pass the ball

### Dribble ball

### Avoid obstacle

### Go to strategic point

### Cover the opponent

### Score the Goal

Strategy
========

Strategy implemented in the team master zero and one.

Output
======

Appreciation of the output.

Future Scope of Improvisation
=============================

Maybe AI can be implemented. Like reinforced learning.

Conclusion
==========

What did we learn.
