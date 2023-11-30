#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def start_up():
  move_group.set_named_target("start_up")
  _ = move_group.plan()
  plan = move_group.go(wait = True)
  return plan


def drop_off():
  group_variable_values = move_group.get_current_joint_values()

  group_variable_values = [1, 1.5, 0, -1.5, -1.5, 0]

  move_group.set_joint_value_target(group_variable_values)

  _ = move_group.plan()
  plan = move_group.go(wait = True)
  return plan


def reach_pickup_position():
    move_group.set_named_target("pick_up_extended")
    _ = move_group.plan()
    plan = move_group.go(wait = True)
    return plan

def tableInit():
  table_pose = geometry_msgs.msg.PoseStamped()
  table_pose.header.frame_id = "g_base"
  table_pose.pose.orientation.w = 1.0
  table_pose.pose.position.x = 0.0
  table_pose.pose.position.y = -0.3
  table_pose.pose.position.z = 0.1  # above the frame
  table_name = "table"
  Scene.add_box(table_name, table_pose, size=(0.1, 0.1, 0.20))


def boxInit():
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "g_base"
  box_pose.pose.orientation.w = 1.0
  box_pose.pose.position.x = -0.04
  box_pose.pose.position.y = 0.29
  box_pose.pose.position.z = 0.0  # above the frame
  box_name = "box"
  Scene.add_box(box_name, box_pose, size=(0.02, 0.02, 0.02))


if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('mycobot_grasping_trial', anonymous=True)

  robot = moveit_commander.RobotCommander()
  Scene = moveit_commander.PlanningSceneInterface()
  group_name = "arm1_mycobot"
  hand_name = "hand_mycobot"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  hand_group = moveit_commander.MoveGroupCommander(hand_name)
  eef_link = hand_group.get_end_effector_link()
  touch_links = robot.get_link_names(group=hand_name)

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
  tableInit()
  boxInit()
  
  plan1 = reach_pickup_position()
  Scene.attach_box(eef_link, "box", touch_links=touch_links)
  plan2 = start_up()
  plan3 = drop_off()
  Scene.remove_attached_object(eef_link, name="box")
  plan4 = start_up()
  
  move_group.stop()
  moveit_commander.roscpp_shutdown()
