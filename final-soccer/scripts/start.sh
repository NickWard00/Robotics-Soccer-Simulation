#!/bin/bash

# The & at the end of each roslaunch command tells the shell to run the command in the background.

# Start the first command in the background
roslaunch ur_gazebo ur5e_bringup.launch &

# Start the second command in the background
roslaunch ur5e_moveit_config moveit_planning_execution.launch sim:=true &