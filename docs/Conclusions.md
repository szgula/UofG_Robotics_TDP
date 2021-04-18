
# Conclusions

 With the environment complete, and the robots engineered, this project can finally be fully wrapped.
 The end product is a simulation of a complete football match following simplified RoboCup guidelines.
 
Both teams compete and one prevails. Team 1 demonstrates great advantage and superior strategy and decision making than Team 0. 
 This structure of having one solid team and one testing team is needed in order to reinforce the algorithms built showing an enhancement in performance (i.e. in Team 1 with respect to Team 0).

Our initial expectations were met using selected architecture, simulation engine and teams controllers. 
The simplified checklist is following:
 - [x] Both teams compete in a quasi-realistic match scenario and maintain interactions without any clashes.
 - [x] Team 0 shows uses semi-advanced strategies and planning.
 - [x] Team 1 demonstrates superior performance over Team 0, coordinating all team members together with the objective of scoring the most points.

It is worth pointed that both teams uses different methodology to control the players: one highly centralised (shared decision tree), one sparse (individual controllers for each robot).
As a result we had an opportunity to observe the benefits and limitations of both approuch. 
Although we do not spend equal amount of development effort on both methodologies, we can conclude the centralised approach gives more promising results, and it can win games with easy.
In comparison, the sparse introduction has greater potential for long term development, due to fewer dependencies between growing code.


This project was created as a proof-of-concept and it needs extra work to become a fully functional robot football simulator. The robots rely on short-term reactions rather than long-term planning. 
 However, to make the robots more robust and less prone to uncertainties in the environment, one can use Reinforcement Learning instead. 
 Given a set of states for each robot, it can dictate which actions maximize its performance measure. 
 Thus, the robot will aim on performing the actions that maximize a certain measure such as lower power utility, or scoring most points...
 <br>With reinforcement learning, a carefully designed environment can certainly accomodate robots or "agents" that cope with the changes in the environment and succeed on finding the optimal actions given a certain performance measure or cost.
 <br>This way, the robots can adapt to any environment in a football match such as snowing, raining, slippery paths. 
 Moreover, they can adapt to the different rules of football such as fouls, penalties tackling and so on.
 <br>Last but not least, the robots can generalize to any given environment changes once designed using Reinforcement Learning.