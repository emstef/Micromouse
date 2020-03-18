##### This project was developed for [COMP513](http://www.intelligence.tuc.gr/~robots/index.html): Autonomous Agents course @[ECE](https://www.ece.tuc.gr/index.php?id=4481) @[TUC](https://www.tuc.gr/).  
# Synopsis
This project is an attempt to develop the Micromouse competition in Webots, a 3D robot simulator, for the purposes of learning the fundamentals of programming an autonomous agent and applying the acquired theoretical knowledge into practice. The implementation was restricted mostly by the complexity of the subject and the limited amount of time for research and development. Consequently, the result of this effort is a simulated autonomous robot that seeks for a path to the centre of a 16x16 block maze. To achieve this goal the robot uses four fundamental principals: localization, mapping, path planning and motion control. While the robot moves in the maze it uses an array of sensors to avoid obstacles and record their position in the maze using as reference its starting position. Simultaneously, the recorded map of the maze is used to determine the possible paths to the centre every time it steps into the next cell.

# What is Micromouse?
One image is worth a thousand words. So this is will spare me some effort:
##### Click to open video (does not open in a new tab)

[![Micromouse](https://i.ytimg.com/vi_webp/NqdZ9wbXt8k/maxresdefault.webp)](http://www.youtube.com/watch?v=NqdZ9wbXt8k "2018 Taiwan Classic micromouse First prize winner")

Micromouse is a contest where a small autonomous robot shall race to the centre of a maze. In the previous video you can watch the micromouse complete 5 runs to the centre. The first run is a search run and the rest are race runs. The mouse has no other information than its starting position (which is always at a corner of the maze with its left side facing the frame of the maze), the size of the maze and that it must reach the centre. In order to calculate the best path to the centre the first run is used to map the maze and it is called a search run. When the mouse makes a race run it might try to explore the maze on its way back to the starting cell.
The maze is meticulously designed to give prominence to the more sophisticated autonomous agents that might take advantage of it in order to minimised their run time. Note that the best path to the centre is not necessarily the shortest because a mouse can go faster when it does not have to turn.
Each Micromouse event more or less has the same set of rules. Differences are found in the scoring system, mainly to promote more advance autonomous behaviour. 

For a better understanding of Micromouse competition read the [“Micromouse Competition Rules”](https://www.ewh.ieee.org/reg/2/sac-18/MicromouseRules.pdf) from a contest held by an IEEE branch.

# What is Webots?
Webots is a free and open-source 3D robot simulator used in industry, education and research. It includes a large collection of freely modifiable models of robots, sensors, actuators and objects and it uses a fork of the ODE (Open Dynamics Engine) for detecting of collisions and simulating rigid body dynamics. The ODE library allows one to accurately simulate physical properties of objects such as velocity, inertia and friction.
The robot controller programs can be written outside of Webots in C, C++, Python, ROS, Java and MATLAB using a simple API. 
##### Source: [Wikipedia](https://en.wikipedia.org/wiki/Webots)

The reason Webots was used in this particular project was that it allowed me to modify an existing demo implementation of the maze based competition called “Rat’s Life”. 
# Approach
This project is divided in two basic development stages: the creation of a virtual environment in Webots, called the world, and the programming of the autonomous agent.
For the Micromouse world, I used as a fundation a built-in Webots demo called “Rat’s Life” which also uses a maze and a robot called e-puck. An implementation that would require the world and the robot model to be build from scratch would have taken a lot more time and effort in a direction that would not comply with the subject of this course. Webots is a very powerful tool but the lack of tutorials and the non existing relative community made it pretty difficult to get started
The programming of the autonomous agent was based upon four fundamental principals: localization, mapping, path planning and motion control. Micromouse does not have a big online community but there are some that try to change that. I am very thankful for that and the are referenced in a following section.
The first part is written in C programming language and the second one in Java, which is something very convenient that Webots supports. There is no other reason in doing so other than that it was inherited from the previous implementation.

# Development
At the first stage, I had to tweak the maze generation implementation of the Rat’s Life demo in order to:
1. Create a 16x16 maze, not a 10x10: This sounds easier than it actually is. Besides changing the values in the relative variables, I had to add each wall via the GUI. That meant I had to create almost 150 walls one by one. I was very lucky I got frustrated and tried every possible way not to do that, and I found out that by editing the world (.wbt) file more walls appeared in the environment. I did the same thing for the links that go between the walls.

2. I wanted to use an [archive](https://github.com/micromouseonline/micromouse_maze_tool/tree/master/mazefiles) that I found that had more than 400 maze designs from previous contests and also some practice designs for, well, practising. So, by overriding the previous generation process I copied an already generated matrix from the archive to the dedicated function. A more elegant way would be to parse the selected file and import the design or even choose a random maze design, but, again, there was not enough time… 
Moreover, the robot always had to start from the same position in any maze, so that was taken care of as well. Anything that was not useful from the demo, like the feeders or the extra e-puck robot, was removed

The last stage of the development was the actual programming of the autonomous agent. I will mention the basic principals behind the logic of the agent but not in detail due to my bare bones implementation and there are plenty of sources that go in depth for each of them.
1. Odometry: 
Using the wheel’s rotary sensors and the IR sensors for correction (simplest possible method)
2. Wall detection: 
Using the IR sensors
3. Localisation and Mapping: 
Using the starting position as a point of reference we can then move through the maze and at each new cell record the surrounding walls.
4. Flood Fill Algorithm: 
Is a search algorithm very useful for path planning in mazes. By assigning the value zero to the destination cell’s weight we can then program the robot to follow the shortest path to the centre.

# Results
This is the Rat's life demo that was used:

![Rat's Life](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/ratslife_1.png)

This is the Micromouse world:

![Microuse](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/Micromouse.png)

And this is the e-puck robot that served the role of the micromouse:

![e-puck](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/epuck.png)
 
These are some samples of the maps that were loaded from the archive:

| ![map1](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/test_maze.png) | ![map2](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/1stworld.png) |
|---|---|
| ![map3](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/map3.png) | ![map4](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/map4.png) |

This is a preview of the mapping array that the mouse constructs while running through the maze:

![cosnole](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/micro_console.gif)
##### Note: Orientation 0: North 1: East 2: South 3: West 

And the final result, the mouse reaching its goal:

![finaly](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/maze_complete.gif)

Press this image to watch the video (does not open in a new tab):
[![Video](https://raw.githubusercontent.com/emstef/Micromouse/master/assets/Micromouse-video.jpg)](https://youtu.be/vqki3yiPmyI "Watch on YouTube")

# References
- [Micromouse Online](http://www.micromouseonline.com/)
- [Micromouse USA](http://www.micromouseusa.com/)
- [Ng Beng Kiat’s site](https://sites.google.com/site/ngbengkiat/)
- [Kato’s Micromouse Wiki](https://seesaawiki.jp/w/robolabo/d/Tetra)
- [Analysis of Micromouse Maze Solving Algorithms](http://web.cecs.pdx.edu/~edam/Reports/2001/DWillardson.pdf)
- [Micromouse Online Github page](https://github.com/micromouseonline)

### Paradox
The last week that the deadline was due I was in Birmingham, UK a few hundrend meters(or yards I should say) away from Birmingham University where the biggest Micromouse competition in Europe takes place every year. I did not even visit because I was a bit behind on the deadline and I had no spare time. 



|Author | Emmanouil Stefanakis|
|--------|----------|
|email| estefanakis1@isc.tuc.gr|
