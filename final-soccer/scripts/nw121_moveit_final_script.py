#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from random import random, uniform
import subprocess, pathlib

class FinalMoveitInterface(object):
    """FinalProjectMoveitInterface"""
    # Main class and object to interface with the robot

    def __init__(self):
        super(FinalMoveitInterface, self).__init__()

        ## Initialize `moveit_commander`_ and a `rospy`_ node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("nw121_moveit_final_script", anonymous=True)

        ## Instantiate a `RobotCommander`_ object to provide information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object to provide a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object to interface can be used to plan and execute motions
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Setting class variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = robot.get_group_names()

        print("Current position values: \n", move_group.get_current_pose().pose)
        print("Current joint values: ", move_group.get_current_joint_values())
    
    def set_initial_position(self, direction):
        # Initializes the robot to the starting position based on if input is right, left, or middle
        # Sets up the move group and the joint goal from the current values
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        if direction == "l": # If left, change configuration to be ready to hit in left direction
            joint_goal[0] = -3.525565089
            joint_goal[1] = -0.872664626
            joint_goal[2] = 1.9896753473
            joint_goal[3] = -1.0995574288
            joint_goal[4] = -0.6457718232
            joint_goal[5] = 0
        elif direction == "r": # If right, change configuration to be ready to hit in right direction
            joint_goal[0] = -3.35
            joint_goal[1] = -(7/25)*pi
            joint_goal[2] = (2/3)*pi
            joint_goal[3] = -(9/25)*pi
            joint_goal[4] = 0
            joint_goal[5] = 0
        elif direction == "m": # If middle, change configuration to be ready to hit in left direction
            joint_goal[0] = -3.57792
            joint_goal[1] = -0.907571
            joint_goal[2] = 2.07694
            joint_goal[3] = -1.27409530718
            joint_goal[4] = -0.401426
            joint_goal[5] = 0

        # Move to those angles using any path within workspace
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

    def kick_ball(self, direction):
        # This is the function responsible for kicking the ball in a certain direction
        # It assumes that the robot is in the initial position
        # It uses helper function move_execute_cartesian to move to each point

        move_group = self.move_group
        # Set the pose to the current pose
        wpose = move_group.get_current_pose().pose
        
        if direction == "m": # If the direction is middle, push the ball forward in x direction by -0.3
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.3)
        elif direction == "l": # If the direction is to the left, push the ball forward in x direction by -0.4 and y by 0.02 to go slightly left
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.4, y_pos=0.02)
        elif direction == "r": # If the direction is to the right, push the ball forward in x direction by -0.3 and y by -0.02 to go slightly right
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.3, y_pos=-0.02)

        # End sequence (get back to initial position and clear targets)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        # Use subprocess module to run the 'gz topic -l' command and capture its output
        topics = subprocess.Popen(('gz', 'topic', '-l'), stdout=subprocess.PIPE, text=True)
        # Use subprocess to filter the output of the previous command using 'grep' (which will come up will all of the references to contacts with the ball 'chassis')
        temp_topic = subprocess.check_output(('grep', '-m', '1', '\/gazebo.*chassis\/contact$'), stdin=topics.stdout, text=True)
        # Remove leading/trailing whitespaces and store the resulting topic
        current_topic = temp_topic.strip()
        print('Collision topic to listen to: ', current_topic)
        # Use subprocess to run 'gz topic -e' command for the extracted topic
        checkNet = subprocess.Popen(('gz', 'topic', '-e', current_topic), stdout=subprocess.PIPE, text=True)
        try:
            # Use subprocess to grep for a line containing 'net' in the output of the previous command
            net_collision = subprocess.check_output(('grep', '-m', '1', '.*net.*'), stdin=checkNet.stdout, timeout=30, text=True)
            # Use subprocess to grep for a line containing 'net' in the output of the previous command
            if net_collision:
                print('\n\nGOAL!!!!!!!!!!!\n\n')
        except subprocess.TimeoutExpired:
            # If the 'grep' command times out, print a no-goal message
            print('\n\nNO GOAL!\n\n')
    
    def move_execute_cartesian(self, move_group, wpose, x_pos=0, y_pos=0, z_pos=0):
        # This function takes in an offset in the direction you want to move the EE
        # It then moves the EE to that position
        waypoints = []
        wpose.position.x += x_pos
        wpose.position.y += y_pos
        wpose.position.z += z_pos

        # Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints.
        # Configurations are computed for every 0.01 meters; 
        # The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. 
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        # Execute the trajectory
        move_group.execute(plan, wait=True)

def main():
    try:
        print("Welcome to Group 5's Soccer Demo using the UR5e!")
        print("Made by: Nick Ward, Quentin MacFarlane, Shane Simkin, and Joe Zakielarz")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")

        char = input("Enter L, M, or R below to hit the ball to the LEFT, MIDDLE, or RIGHT\n").strip().lower()
        print("Selected: " + char)
        move_it = FinalMoveitInterface()

        if char == "l" or char == "r" or char == "m":
            move_it.set_initial_position(char)
        else:
            print("Invalid character...")
            return
  
        move_it.kick_ball(char)

        input(
            "============ Press `Enter` to get a new goalie and ball..."
        )
        # Use subprocess.run to spawn a new football model in Gazebo
        spawnInNewBall = subprocess.run(['rosrun', 'gazebo_ros', 'spawn_model', '-file', str(pathlib.Path.home()) + '/final-soccer/models/football/model.sdf', '-sdf', '-z', '2', '-model', str(random())])
        # Use subprocess.run to spawn a new goalie model in Gazebo. 
        # Its x and z positions are fixed, but the y value can be anything between -0.6 to 0.6, which is the limits of each goal post.
        spawnInNewGoalie = subprocess.run(['rosrun', 'gazebo_ros', 'spawn_model', '-file', str(pathlib.Path.home()) + '/final-soccer/models/Placeholder_Goalie/model.sdf', '-sdf', '-x', '-4.20', '-y', str(uniform(-0.6, 0.6)), '-z', '0.286', '-model', str(random())])

        print("============ Everything successfully executed! Run the script again to kick again!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()