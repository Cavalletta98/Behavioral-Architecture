# Behavioral-Architecture
A robot that interact with a human and moves in a discrete 2D environment

# Software architecture and states diagrams
## Software architecture 
The architercture is composed by 4 components: 

    1. Command: simulate user vocal command

    2. Gesture: simulate user pointed gestures

    3. Motion: simulate robot motion

    4. Command manager: implement robot behaviors through a FSM

## State diagram
The finite state machine is composed by 3 state (robot behaviours):

    1. PLAY: robot moves from person position to pointed gestures and      viceversa

    2. NORMAL (initial state): robot moves in a random ways

    3. SLEEP: robot goes to home position and then goes to NORMAL state

## ROS messages and parameters
The messages are:

- `Point`: 2D position used for pointed gestures and target position
- `String`: used for user command and feedback from "motion" component

The parameters are:

- `home_pos_x,home_pos_x`: define the home position in the map
- `person_pos_x,person_pos_y`: define the person  position in the map 
- `map_x,map_y`: define the dimensions of the map

# Packages and files
There are 3 packages:

- `Sensoring`: contains the [command.py](sensoring/src/command.py) and [gesture.py](sensoring/src/gesture.py) files used to simulate the user command and pointed gestures
- `Robot control`: contains the [motion.py](robot_control/src/motion.py) file used to simulate robot motion
- `Command manager`: contains the [command_manager.py](manager/src/command_manager) file that implements the FSM of robot behaviours

# Installation and running
In order to run this software, the following prerequisities are needed:
- [ROS Noetic](http://wiki.ros.org/noetic)
- [smach](http://wiki.ros.org/smach)

Before running the software, you must have all files as executable otherwise you can make them executable with the following command
```
cd <your_workspace>/src/Behavioral-Architecture
chmod +x sensoring/src/*
chmod +x robot_control/src/*
chmod +x manager/src/*
```
To run the software
```
cd <your_workspace>/src/Behavioral-Architecture
roslaunch launch_file.launch
```

# Working hypothesis and environment
The map is a descrite cartesian space where you can specify the dimensions for x and y axes.

# System's features
Once the map, home and person position are defined, the robot starts from the initial state (NORMAL) and moves in all the generated positions and it makes transitions between states.

# System's limitations
Since the user command is implemented as a simulated one sended with random delays, it can happens that the robot often makes a transition between NORMAL and PLAY states without going in the SLEEP state (fewer times than the other transition).

# Author
@Cavalletta98 - simone.voto98@gmail.com