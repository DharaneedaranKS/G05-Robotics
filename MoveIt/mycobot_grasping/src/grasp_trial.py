#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mycobot_grasping_trial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm1_mycobot"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)

# Set up the planning scene, if needed
# ... (add scene setup code here if necessary)

# Set the planning time
move_group.set_planning_time(5.0)  # You can adjust this value based on your robot's complexity

# Define the goal pose
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 1
pose_goal.position.y = 3
pose_goal.position.z = 2

# Set the goal pose
move_group.set_pose_target(pose_goal)

# Plan the trajectory
plan_success, plan, planning_time, error_code = move_group.plan()

# Check if planning was successful
if not plan:
    rospy.logerr("Planning failed. Exiting...")
    moveit_commander.roscpp_shutdown()
    sys.exit(1)

# Visualize the trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
"""
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_publisher.publish(display_trajectory)

# Execute the trajectory
# move_group.execute(plan, wait=True)

# Wait for a moment to allow the execution to finish
rospy.sleep(5)

# Shutdown MoveIt! commander
# moveit_commander.roscpp_shutdown()
"""