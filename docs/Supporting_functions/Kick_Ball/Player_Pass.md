
# **Actions**

**[Go back to main page](../../../Documentation.md)**

## Player Pass
This algorithm is responsible for finding the best mate to pass the ball to. 
It calculates if an opponent can capture the passing ball, suggests a player to pass and provide an interface for "collaboration awareness" used by strategy makers.

 ## Implementation

 To generate an efficient passing method, players must process information about the environment.
 The main problem the algorithm needs to solve is whether it can pass the ball or not. 
 Passing the ball follows a careful procedure of checking which players are candidates for a feasible calculated pass.

 In this process following inputs are needed:
1. Team member positions - the ball holder needs to know all positions of his teammates in order to decide if to pass and to which target.
2. Opponent positions - in order to establish whether a pass is feasible or not, the robot needs the opponents' positions in order to check if they might be in the area and trajectory of the ball once passed.
3. Ball state - information about the ball's position and velocity. 

The algorithm contains three main steps:
1. Check if pass is feasible for all teammates.
2. Calculate teammates pass priority based on their distance from the net.
3. Identify an optimal candidate to pass the ball to.

Consider the following case study:
Player A has the ball and decides on passing since no opponents are around. 
The closest player to the net Player B is selected as the best candidate to pass the ball to. 
This is presented in Figure 1:

<p align="center">
  <img src="../../../Images/pass_capabilities.png" />
</p>

__Figure 1__: Pass candidates diagram.

 Player A's logical action is to pass to Player B based on what is depicted in Figure 1.
 Since three is not opponents in a danger zone, no opponent can capture the pass. 
 Player B is chosen as a candidate since it is the closest player to the net, hence all other players are neglected.
 Hence, the robots are going to act aggressively instead of thoroughly planning and taking into account all possibilities.

Deciding if a pass is feasible is up to the triangles emitted by the player. These triangles contain three straight lines that dictate danger levels. 
The left-most line poses the most risk. The further two are medium and least risks respectively.

 
Figure 2 shows passing algorithm aspects explained into one short animation reinforcing the decisions a player takes once involved in passing.
It demonstrates how player 3 decided to pass to teammate 2 instead of player 1. 

<p align="center">
     <img src="../../../Images/pass_algo.gif" />
</p>

__Figure 2__: Passing Algorithm Example
