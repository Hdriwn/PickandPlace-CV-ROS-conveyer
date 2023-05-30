# pickplace-CV-conveyer-ROS
## Conveyor Pick and Place Project

This project focuses on implementing a pick and place system using a conveyor belt. The goal is to develop a robotic manipulator that can pick up moving boxes from the conveyor, place them on the ground, and keep track of their positions.

**Note: This project is designed for ROS Melodic on Ubuntu 18.04.**

## Installation

Clone this package by running the following command in your terminal:

```bash

git clone git@github.com:Hdriwn/pickplace-CV-conveyer-ROS.git
```

## Part 1: Pick and Place Using Conveyor

### Launching the System

1. Build the package using Catkin:
```bash
catkin_make
```
2.Launch the Gazebo simulation:
```bash
roslaunch pickplace demo_gazebo.launch
```
3,Activate the conveyor belt by running the following command in a new terminal tab:
```bash
rosservice call /conveyor/control "power: 10.0"
```
4. Now spawn the blocks
```bash
rosrun spawn_urdf_sdf spawn
```
5. Running the Control Program

To run the control program, execute the following command:
```bash
rosrun pickplace pickplace_cc
```
## Part 2: Robotic Manipulator Along Conveyor 
This part focuses on a robotic manipulator that moves along the conveyor and picks a specified box from it. The system keeps track of the position of every box on the conveyor.
### Launching the System
Launch the Gazebo simulation and activate the conveyor belt as in Part 1.
```bash
roslaunch slider_pickplace demo_gazebo.launch
```
2. Running the Control Program
To run the control program, execute the following command:
```bash
rosrun slider_pickplace slider
```
To pick a specific object and track its current position, set the ID of the object using the service. For example:
```bash
rosservice call /user_request "requested_value: 1"
```
Please note that this project is still a work in progress, and part 2 is yet to work .

