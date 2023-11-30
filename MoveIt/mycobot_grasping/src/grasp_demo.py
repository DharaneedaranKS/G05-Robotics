#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math 

def start_up() :
  move_group.set_named_target("start_up")
  plan_start_up = move_group.plan()
  plan = move_group.go(wait = True)
  # move_group.execute(plan)
  return plan

def drop_off() :
  group_variable_values = move_group.get_current_joint_values()

  group_variable_values = [1, 1.5, 0, -1.5, -1.5, 0]

  move_group.set_joint_value_target(group_variable_values)

  plan1 = move_group.plan()
  plan = move_group.go(wait = True)
  return plan
  # move_group.execute(plan2)

"""def gripper_close() :
  hand_variable_values = hand_group.get_current_joint_values()
  print(hand_variable_values)
  hand_variable_values[0] = -0.21
  #hand_group.set_joint_value_target("gripper_controller", hand_variable_values)
  #planx = hand_group.plan()
  hand_group.go(hand_variable_values, wait = True)
  hand_group.stop()"""

 

def reach_named_position():
    move_group.set_named_target("pick_up_extended")
    plan_pick_up = move_group.plan()
    plan = move_group.go(wait = True)
    #move_group.execute(plan)
    return plan

def table() :
  table_pose = geometry_msgs.msg.PoseStamped()
  table_pose.header.frame_id = "g_base"
  table_pose.pose.orientation.w = 1.0
  table_pose.pose.position.x = 0.0
  table_pose.pose.position.y = -0.3
  table_pose.pose.position.z = 0.1  # above the frame
  table_name = "table"
  SCENE.add_box(table_name, table_pose, size=(0.1, 0.1, 0.20))

def box() :
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "g_base"
  box_pose.pose.orientation.w = 1.0
  box_pose.pose.position.x = -0.04
  box_pose.pose.position.y = 0.29
  box_pose.pose.position.z = 0.0  # above the frame
  box_name = "box"
  SCENE.add_box(box_name, box_pose, size=(0.02, 0.02, 0.02))


if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('mycobot_grasping_trial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  SCENE = moveit_commander.PlanningSceneInterface()
  group_name = "arm1_mycobot"
  hand_name = "hand_mycobot"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  hand_group = moveit_commander.MoveGroupCommander(hand_name)
  eef_link = hand_group.get_end_effector_link()
  touch_links = robot.get_link_names(group=hand_name)

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
  table()
  box()
  
  #start_up()
  #gripper_close()
  plan1 = reach_named_position()
  
  SCENE.attach_box(eef_link, "box", touch_links=touch_links)
  plan2 = start_up()

  plan3 = drop_off()
  SCENE.remove_attached_object(eef_link, name="box")
  plan4 = start_up()
  move_group.stop()
  moveit_commander.roscpp_shutdown()

# move_group.execute(plan)
