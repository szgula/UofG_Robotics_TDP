Introduction
============

Why such robo games are important. What we can hope to achive and learn
from implementing such projects. More than just a game.

Problem Statement
-----------------

Problem Statement given by McGookin. His expectations mentioned by him
in the beginning.

Objective
---------

Since the Problem Statement gave us the freedom to decide the scope of
the project. The scope must be iterated here. We must also justify the
use of ROS, Python etc. We must also justify the lowest level of
abstraction here. We assume here that the controllers can impart an
instantaneous angulat velocity to the wheels.

Overview
--------

Here a bird’s eye of the Project and upcoming chapters should be given.

ROS nodes
=========

Since ROS implementation gives the structure to the project at the
highest level it must be discussed first. Brief description of what is
ROS. How it has been implemened.

Physics
=======

In this section the coordinate system of the field and the physics of
the simulation is discussed.

Coordinate System
-----------------

Coordinate System of the football field. Global and Ego-Coordinate
systems, why two Co-ordinate Systems were used. The size of the field is
10 units in horizontal direction and six units in the vertical
direction. The origin lies at the centre of the field which makes x lie
between -5 and 5 and y lie between -3 and 3.

Kinematics
----------

### Step

The position of Robots and the ball are kept in a buffer and updated in
syncronization with each step in the simulation. All robots and the ball
have a common clock.

### Ball Model

#### Friction

Friction is implemented in each step.

#### Collision

#### Players Action

##### Kick

##### Receive

### Robot Model

Differential drive Kinematics and how it was implemented.
$$x_n = x + (r_w  \Delta t / 2) (\omega_l + \omega_r)$$

Basic Queries
=============

For calculating the next action, the robots need to know the position of
the ball and all the players on the field. For this the players can
query into the state of the simulator using various functions. Such
functions are very important in the implementation of the decision tree
as discussed in the next chapter.

Obstacle distance calculation
-----------------------------

Has ball or not
---------------

Check if the ball is free
-------------------------

This function gives the value of True when the ball is outside the
proximity zone of all the players on the field.

Check if the ball is in the teams half
--------------------------------------

Check if goal can be scored or not
----------------------------------

Check if dribble is safe or not
-------------------------------

Check for pass
--------------

Check if the ball can be passed by a defender to another
--------------------------------------------------------

Check which player can reach fastest to the ball
------------------------------------------------

Check which striker of the team is closest
------------------------------------------

Basic Actions
=============

Atomic actions such as going to a point, scoring the goal, passing the
ball, collision avoidance etc.

Go to point
-----------

Go around a point
-----------------

Rotate towards a direction
--------------------------

Pass the ball
-------------

Dribble ball
------------

Avoid obstacle
--------------

Go to strategic point
---------------------

Cover the opponent
------------------

Score the Goal
--------------

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
