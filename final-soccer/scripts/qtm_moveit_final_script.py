#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from std_srvs.srv import Empty
from math import pi
from random import random
import subprocess

class FinalMoveitInterface(object):
    """MidtermMoveitInterface"""
    # Main class and object to interface with the robot

    def __init__(self):
        super(FinalMoveitInterface, self).__init__()

        ## Initialize `moveit_commander`_ and a `rospy`_ node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("srs105_moveit_final_script", anonymous=True)

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

        print("Current position values: ", move_group.get_current_pose().pose)
        print("Current joint values: ", move_group.get_current_joint_values())
    
    # Initializes the robot to kick the ball to the LEFT
    def initial_position_left(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        
        joint_goal[0] = -3.6
        joint_goal[1] = -(7/25)*pi
        joint_goal[2] = (2/3)*pi
        joint_goal[3] = -(9/25)*pi
        joint_goal[4] = 2.5 - pi
        joint_goal[5] = (7/10)*pi

        # Move to those angles using any path within workspace
        move_group.go(joint_goal, wait=True)

    # Initializes the robot to kick the ball to the RIGHT
    def initial_position_right(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        
        joint_goal[0] = -3.35
        joint_goal[1] = -(7/25)*pi
        joint_goal[2] = (2/3)*pi
        joint_goal[3] = -(9/25)*pi
        joint_goal[4] = 0
        joint_goal[5] = (7/10)*pi

        # Move to those angles using any path within workspace
        move_group.go(joint_goal, wait=True)

    # Initializes the robot to kick the ball to the MIDDLE
    def initial_position_middle(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()

        joint_goal[0] = -3.45
        joint_goal[1] = -(7/25)*pi
        joint_goal[2] = (2/3)*pi 
        joint_goal[3] = -(9/25)*pi
        joint_goal[4] = 1.3 - pi/2
        joint_goal[5] = (7/10)*pi

        move_group.go(joint_goal, wait=True)

    def kick_ball(self, direction):
        # This is the function responsible for drawing the N
        # It assumes that the robot is in the initial position
        # It uses helper function move_execute_cartesian to move to each point

        move_group = self.move_group
        # Set the pose to the current pose
        wpose = move_group.get_current_pose().pose
        
        # Point 1
        if direction == "m":
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.4)
        elif direction == "l":
            # TODO: figure out these values
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.4, y_pos=0.02)
        else:
            # TODO: figure out these values
            self.move_execute_cartesian(move_group, wpose, x_pos=-0.4, y_pos=-0.01)

        
        topics = subprocess.Popen(('gz', 'topic', '-l'), stdout=subprocess.PIPE, text=True)
        temp_topic = subprocess.check_output(('grep', '-m', '1', '\/gazebo.*chassis\/contact$'), stdin=topics.stdout, text=True)
        current_topic = temp_topic.strip()
        print('Collision topic to listen to: ', current_topic)
        # if 'football' not in current_topic:
        checkNet = subprocess.Popen(('gz', 'topic', '-e', current_topic), stdout=subprocess.PIPE, text=True)
        net_collision = subprocess.check_output(('grep', '-m', '1', '.*net.*'), stdin=checkNet.stdout, timeout=70, text=True)
        if net_collision:
            print('\n\nGOAL!!!!!!!!!!!\n\n')
                
        # End sequence (get back to initial position and clear targets)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    
    def reset_simulation(self):
        reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset()
    
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
        print("")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        # input(
        #     "============ Press `Enter` to set up the moveit_commander ..."
        # )

        char = input("Press L, M, or R\n").strip().lower()
        print("You selected:", char)
        move_it = FinalMoveitInterface()

        if char == "l":
            move_it.initial_position_left()
        elif char == "m":
            move_it.initial_position_middle()
        elif char == "r":
            move_it.initial_position_right()
        else:
            print("Invalid character...")
            #move_it.reset_simulation()
            return

        # move_it.initial_position()
  
        move_it.kick_ball(char)

        input(
            "============ Press `Enter` to get to reset simulation..."
        )
        #move_it.reset_simulation()
        result = subprocess.run(['rosrun', 'gazebo_ros', 'spawn_model', '-file', 'models/football/model.sdf', '-sdf', '-z', '2', '-model', str(random())])
        #

        print("============ Everything successfully executed!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()