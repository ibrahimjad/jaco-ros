# jaco-ros

**ROS package for controlling Jaco2 robot arm**

## Prerequisites
* [kinova-ros](https://github.com/Kinovarobotics/kinova-ros/)
* gtkmm-3.0

## Installation
Clone this repository to your workspace and then build it using either **catkin_make** or **catkin build**.

## Usage
Type the following command to run jaco_control node:
```bash
rosrun jaco_control bringup
```
Type the following command to run jaco_teach node:
```bash
rosrun jaco_teach bringup
```
