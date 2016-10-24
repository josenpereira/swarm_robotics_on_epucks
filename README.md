# swarm_robotics_on_epucks
e-puck controller for swarm robotics case study

This code was implement in 2012 to replicate the original wireless connected swarm case study
with real e-puck robots (equipped with an external zigbee radio module). This is a simple
swarm robotics case study concerned with maintaining a swarm of robots in close proximity (while
the robot are moving forward and avoiding obstacles) by using solely basic communication.
Robots broadcast their ids at a given time interval and listen to messages from other robots.
The decisions of the robots are based solely on the number of messages received. Each robot 
executes a copy of this controller.

A video of the experiment using 40 robots can be seen at:
https://www.youtube.com/watch?v=zKw1bwccnKA

This simple C code takes advantage of the timers of the e-puck dsPic microprocessor to decide
when to broadcast information to its neighbors.
