#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def start_up() :
  move_group.set_named_target("start_up")
  plan_start_up = move_group.plan()
  move_group.go(wait = True)

def pick_up() :
  group_variable_values = move_group.get_current_joint_values()

  group_variable_values = [1, 1.5, 0, -1.5, -1.5, 0]

  move_group.set_joint_value_target(group_variable_values)

  plan1 = move_group.plan()
  plan2 = move_group.go(wait = True)

def gripper_close() :
  hand_variable_values = hand_group.get_current_joint_values()
  #print(hand_variable_values)
  hand_variable_values = [-0.166, 0.166, 0.166, -0.166, -0.166, 0.166]
  hand_group.set_joint_value_target(hand_variable_values)
  planx = hand_group.plan()
  planx2 = hand_group.go(wait = True)

def pick_by_position():
  move_group.set_position_target([0.2, 0.2, 0.1], "joint6")

def reach_named_position():
    move_group.set_named_target("pick_up_extended")
    return move_group.execute(move_group.plan()[1], wait=True)


if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('mycobot_grasping_trial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  SCENE = moveit_commander.PlanningSceneInterface()
  group_name = "arm1_mycobot"
  hand_name = "hand_mycobot"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  hand_group = moveit_commander.MoveGroupCommander(hand_name)

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
  
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "g_base"
  box_pose.pose.orientation.w = 1.0
  box_pose.pose.position.x = 0.0
  box_pose.pose.position.y = -0.3
  box_pose.pose.position.z = 0.1  # above the frame
  box_name = "box"
  SCENE.add_box(box_name, box_pose, size=(0.1, 0.1, 0.20))
  
  start_up()
  gripper_close()
  #reach_named_position()

  #pick_up()

  #start_up()

  #pick_by_position()

  # start_up()

  moveit_commander.roscpp_shutdown()

# move_group.execute(plan)
