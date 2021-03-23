# **Strategy**

**[Go back to main page](../Documentation.md)**

## Decision Making

Once all the structure was implemented in terms of ROS and simulations, a decision making process needed to be implemented. Two of them were developped and compared:

1. Hard coded rules for each player.
2. Decision tree for all players (excluding goal keeper as single controller).

The first one is as simple as it is. Each player has specfifc locations to follow and specific actions to perform. This methodology has been used to test the more appropriate strategies upon.

The second approach however, combines all players and decides on appropriate actions based on the decision tree structure. This method has been agreed on since it is simple and straightforward. The tree behaves like a coach for the team. It gathers information from the simulator related to the team itself and the opponents and then narrows down that information into a tuple of actions. 

The tree's structure is divided into two main branches, one that is based on the player having the ball, and one that is based on the player not having the ball. The diagrams for both of these branches are as follows:

1. If the player has the ball:

<p align="center">
  <img src="../Images/decesionTree.png" />
</p>

2. If the player does not have the ball:

   <p align="center">
     <img src="../Images/decisionTreeTwo.png" />
   </p>

The decision tree sets an importance to has_ball() since the goal of the team is to get the most goals and win the match. Therefore, the first node that the tree follows is if the player has the ball. If yes, then follow the flow of the first branch, if not then shift to the next one.

To be able to differentiate between one player and another, the tree includes a bunch of information about the players. First off, the tree acquires the player ids and their corresponding modes. The former are unique ids for each player in the game. The latter are two options for a pair of players that can either be "DEFEND" or "ATTACK". With this seperation between the players, the decision tree will accurately know which actions to assign to which player.

An example of the flow that the tree follows can be found below:

<p align="center">
  <img src="../Images/Decision Flow.png" />
</p>

