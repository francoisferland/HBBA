#!/usr/bin/env bash 

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot_hbba_cfg turtlebot_gazebo.launch &
roslaunch turtlebot_hbba_cfg turtlebot_nav_slam.launch
