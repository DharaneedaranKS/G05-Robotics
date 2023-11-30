#!/usr/bin/env python3
"""
EE430 G05
==========

Authors:
-------
Kulkarni Hrishikesh (201EE101)
Adithyan Mundayadu (201EE103)
Dharaneedaran K S (201EE120)
Kalluru Arif (201EE125)
"""

import rospy
from std_msgs.msg import Float64
import numpy as np
import math
from control_msgs.msg import JointControllerState
from sympy import symbols, cos, sin, pi, Matrix
from g05 import *


def velocityMatrix(error):
    """
    Calculate the x_dot matrix

    Parameters: error (desired position - current pose)

    Returns:
        Matrix: x_dot matrix (with respect to base frame)
    """
    Vx, Vy, Vz = error
    Wx = Wy = Wz = 0
    return Matrix([[Vx], [Vy], [Vz], [Wx], [Wy], [Wz]])


def jointVelocityMatrix(J, qk, error):
    """
    Find the joint velocity matrix

    Parameters: J (Jacobian matrix), qk (kth joint variables), error (desired position - current pose)

    Returns:
        q_dot (Matrix)
    """
    X_dot = velocityMatrix(error)

    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    q1, q2, q3, q4, q5, q6 = qk

    J = J.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
    # J_inverse = J.inv() # You'll be waiting for centuries! [Reason: inverting sympy symbolic matrices takes time]

    # Convert Jacobian matrix to numpy array for numerical computations
    J_numpy = np.array(J).astype(float)

    # Calculate the inverse of the Jacobian matrix using numpy
    J_inv_numpy = np.linalg.inv(J_numpy)

    # Convert the numpy inverse back to a SymPy matrix
    J_inv = Matrix(J_inv_numpy)

    # Calculate q_dot using the inverted Jacobian matrix
    q_dot = J_inv * X_dot
    return q_dot


def move_to_point(o_n, J):
    # Initial angles (chosen arbitrarily)
    q0 = Matrix([[-math.pi/2], [-3*math.pi/4], [-3*math.pi/4], [0], [math.pi/2], [0]])

    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    q1, q2, q3, q4, q5, q6 = q0
    o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])

    x0, y0, z0 = o_n_numerical
    publish_values(q0)

    current_pose = get_pose(o_n, qk_process_value)

    print("Input desired x, y, z positions (in mm): ")
    x1 = int(input())
    y1 = int(input())
    z1 = int(input())
    
    qk = q0
    num_steps = 100
    time_taken = 5
    delta_t = time_taken / num_steps
    waypoints = np.linspace((x0, y0, z0), (x1, y1, z1), num=num_steps)

    Kp = 6

    for i in range(num_steps):
        current_pose = get_pose(o_n, qk_process_value)
        error = waypoints[i] - current_pose
        qk_plus_1_dot = jointVelocityMatrix(J, qk, Kp * error)
        qk_plus_1 = qk + qk_plus_1_dot * delta_t
        qk = qk_plus_1 % (2*math.pi)
        
        # End-effector position
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        q1, q2, q3, q4, q5, q6 = qk
        o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
        publish_values(qk)


def joint1_pos_callback(data):
    qk_process_value[0] = data.process_value


def joint2_pos_callback(data):
    qk_process_value[1] = data.process_value


def joint3_pos_callback(data):
    qk_process_value[2] = data.process_value


def joint4_pos_callback(data):
    qk_process_value[3] = data.process_value


def joint5_pos_callback(data):
    qk_process_value[4] = data.process_value


def joint6_pos_callback(data):
    qk_process_value[5] = data.process_value


def reset_robot():
    rate = rospy.Rate(6)
    pub_joint1_pos.publish(0)
    rate.sleep()
    pub_joint2_pos.publish(0)
    rate.sleep()
    pub_joint3_pos.publish(0)
    rate.sleep()
    pub_joint4_pos.publish(0)
    rate.sleep()
    pub_joint5_pos.publish(math.pi/2)
    rate.sleep()
    pub_joint6_pos.publish(0)
    rate.sleep()


def publish_values(qk):
    rate = rospy.Rate(500)

    q1, q2, q3, q4, q5, q6 = qk

    pub_joint1_pos.publish(q1 + math.pi/2)
    pub_joint2_pos.publish(q2 + math.pi/2)
    pub_joint3_pos.publish(q3 + math.pi/2)
    pub_joint4_pos.publish(q4)
    pub_joint5_pos.publish(q5)
    pub_joint6_pos.publish(q6)
    rate.sleep()


# Get current real pose from Gazebo
def get_pose(o_n, qk_process_value):
    # Ensure joint angles are within [0, 2*pi)
    q1, q2, q3, q4, q5, q6 = (q % (2*math.pi) for q in qk_process_value)

    # Update joint angles
    qk_process_value = q1, q2, q3, q4, q5, q6

    # Adjust joint angles
    q1 -= math.pi/2
    q2 -= math.pi/2
    q3 -= math.pi/2

    # Symbolic joint variables
    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')

    # Substitute joint angles into symbolic end-effector position
    o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])

    # Extract current pose
    current_pose = [o_n_numerical[0], o_n_numerical[1], o_n_numerical[2]]

    return current_pose


if __name__ == '__main__':
    try:

        rospy.init_node('move_robot', anonymous=False)
        o_n, J = g05()

        pub_joint1_pos = rospy.Publisher('/g05/Revolute_1_position_controller/command', Float64, queue_size=500) 
        pub_joint2_pos = rospy.Publisher('/g05/Revolute_2_position_controller/command', Float64, queue_size=500)
        pub_joint3_pos = rospy.Publisher('/g05/Revolute_3_position_controller/command', Float64, queue_size=500)
        pub_joint4_pos = rospy.Publisher('/g05/Revolute_4_position_controller/command', Float64, queue_size=500)
        pub_joint5_pos = rospy.Publisher('/g05/Revolute_5_position_controller/command', Float64, queue_size=500)
        pub_joint6_pos = rospy.Publisher('/g05/Revolute_6_position_controller/command', Float64, queue_size=500)

        joint1_pos= rospy.Subscriber('/g05/Revolute_1_position_controller/state',JointControllerState, joint1_pos_callback)
        joint2_pos= rospy.Subscriber('/g05/Revolute_2_position_controller/state',JointControllerState, joint2_pos_callback)
        joint3_pos= rospy.Subscriber('/g05/Revolute_3_position_controller/state',JointControllerState, joint3_pos_callback)
        joint4_pos= rospy.Subscriber('/g05/Revolute_4_position_controller/state',JointControllerState, joint4_pos_callback)
        joint5_pos= rospy.Subscriber('/g05/Revolute_5_position_controller/state',JointControllerState, joint5_pos_callback)
        joint6_pos= rospy.Subscriber('/g05/Revolute_6_position_controller/state',JointControllerState, joint6_pos_callback)

        qk_process_value = [0, 0, 0, 0, 0, 0]  # initial joint angles from robot in Gazebo
        reset_robot()
        move_to_point(o_n, J)

    except rospy.ROSInterruptException: 
        pass