# Conclusions

 With the developed environment, and the robots engineered, this project can finally be fully wrapped.
 The end product is a simulation of a complete football match that follows simplified RoboCup guidelines.
 
Both teams compete against each other and only one team wins. Team 1 demonstrates greater advantage, superior strategy and decision making over Team 0. 
 This structure of having one solid team and one testing team is needed in order to reinforce the algorithms and show the performance improvement of Team 1.

Our initial expectations were met using selected architecture, simulation engine and teams controllers. 
The simplified checklist is the following:
 - [x] Both teams compete in a quasi-realistic match scenario and maintain interactions without any clashes.
 - [x] Team 0 performs semi-advanced strategies and planning.
 - [x] Team 1 demonstrates superior performance over Team 0, coordinates all team members with the objective to score the points.

It is worth pointing that both teams use different methodologies to control the players: one is highly centralised (shared decision tree), and one is sparse (individual controllers for each robot).
As a result, we had an opportunity to observe the benefits and limitations of both approaches. 
Although we did not spend an equal amount of development effort on both methodologies, we can conclude that the centralised approach gives more promising results, and it can win games with ease.
In contrast, the sparse architecture has greater potential for long term development due to fewer dependencies between growing code.


This project was created as a proof-of-concept and it needs extra work to become a fully functional simulator for robotic football. With the current implementation, robots rely on short-term reactions rather than long-term planning. 
 However, to make the robots more robust and less susceptible to uncertainties in the environment, one can use Reinforcement Learning (RL) instead. 
 Given a set of states for each robot, RL can compute which actions maximise its performance measure. 
 Thus, the robot will aim to perform the actions that maximise a certain measure, such as lower power utility, or scoring most points.
 
With RL, a carefully designed environment can certainly accommodate robots or "agents" that cope with the changes in the environment and succeed in finding the optimal actions given a certain performance measure or cost.
With this approach, the robots can adapt to any environmental condition in a football match, such as snowing, raining, slippery paths. 
 Moreover, they can adapt to the different rules of football, such as fouls, penalties tackling and so on.
 Last but not least, the robots can generalise to any given environment changes once designed using RL.