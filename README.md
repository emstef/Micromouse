##### This project was developed for [COMP513](http://www.intelligence.tuc.gr/~robots/index.html): Autonomous Agents course @[ECE](https://www.ece.tuc.gr/index.php?id=4481) @[TUC](https://www.tuc.gr/).  
# Synopsis
This project is an attempt to develop the Micromouse competition in Webots, a 3D robot simulator, for the purposes of learning the fundamentals of programming an autonomous agent and applying the acquired theoretical knowledge into practice. The implementation was restricted mostly by the complexity of the subject and the limited amount of time for research and development. Consequently, the result of this effort is a simulated autonomous robot that seeks for a path to the centre of a 16x16 block maze. To achieve this goal the robot uses four fundamental principals: localization, mapping, path planning and motion control. While the robot moves in the maze it uses an array of sensors to avoid obstacles and record their position in the maze using as reference its starting position. Simultaneously, the recorded map of the maze is used to determine the possible paths to the centre every time it steps into the next cell.

# What is Micromouse?
One image is worth a thousand words. So this is will spare me some effort:

[![Micromouse](https://i.ytimg.com/vi_webp/NqdZ9wbXt8k/maxresdefault.webp)](http://www.youtube.com/watch?v=NqdZ9wbXt8k "2018 Taiwan Classic micromouse First prize winner" =100x200)

Micromouse is a contest where a small autonomous robot shall race to the centre of a maze. In the previous video you can watch the micromouse complete 5 runs to the centre. The first run is a search run and the rest are race runs. The mouse has no other information than its starting position (which is always at a corner of the maze with its left side facing the frame of the maze), the size of the maze and that it must reach the centre. In order to calculate the best path to the centre the first run is used to map the maze and it is called a search run. When the mouse makes a race run it might try to explore the maze on its way back to the starting cell.
The maze is meticulously designed to give prominence to the more sophisticated autonomous agents that might take advantage of it in order to minimised their run time. Note that the best path to the centre is not necessarily the shortest because a mouse can go faster when it does not have to turn.
Each Micromouse event more or less has the same set of rules. Differences are found in the scoring system, mainly to promote more advance autonomous behaviour. 

For a better understanding of Micromouse competition read the [“Micromouse Competition Rules”](https://www.ewh.ieee.org/reg/2/sac-18/MicromouseRules.pdf) from a contest held by an IEEE branch.

# What is Webots?
Webots is a free and open-source 3D robot simulator used in industry, education and research. It includes a large collection of freely modifiable models of robots, sensors, actuators and objects and it uses a fork of the ODE (Open Dynamics Engine) for detecting of collisions and simulating rigid body dynamics. The ODE library allows one to accurately simulate physical properties of objects such as velocity, inertia and friction.
The robot controller programs can be written outside of Webots in C, C++, Python, ROS, Java and MATLAB using a simple API. 
##### Source: [Wikipedia](https://en.wikipedia.org/wiki/Webots)

The reason Webots was used in this particular project was that it allowed me to modify an existing demo implementation of the maze based competition called “Rat’s Life”. 
# Aproach

# Development

# Results






## Micromouse in Webots Project 
Author: Emmanouil Stefanakis

Micromouse is a competition where a small mouse robot solves a 16x16 block maze.
#### [COMP513]

### Micromouse
1. Maze Solving: 
Moving/searching around for the best path to reach the center. Use of searching algorithms to compute shortest path.

2. Performance: 
The shortest path is not always the fastest. Straight lines enable the mouse to accelerate.

### Webots
Webots is a professional robot simulator widely used for educational purposes.

