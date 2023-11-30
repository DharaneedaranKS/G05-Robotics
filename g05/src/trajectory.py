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
from sympy import symbols, Matrix
from g05 import *


def velocityMatrix():
    """
    Calculate the x_dot matrix

    Parameters: theta (polar parameter)

    Returns:
        Matrix: x_dot matrix (with respect to base frame)
    """

    Vy = 100
    Vz = 52.9
    Vx = Wx = Wy = Wz = 0
    return Matrix([[Vx], [Vy], [Vz], [Wx], [Wy], [Wz]])


def jointVelocityMatrix(J, qk):
    """
    Find the joint velocity matrix

    Parameters: theta (polar parameter), J (Jacobian matrix), qk (kth joint variables)

    Returns:
        q_dot (Matrix)
    """
    X_dot = velocityMatrix()

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


def follow_trajectory(o_n, J):
    q0 = Matrix([[-math.pi/2], [-3*math.pi/4], [-3*math.pi/4], [0], [math.pi/2], [0]])
    qk = q0
    num_steps = 100
    time_taken = 5
    delta_t = time_taken / num_steps

    for _ in range(num_steps + 1):
        qk_plus_1_dot = jointVelocityMatrix(J, qk)

        qk_plus_1 = qk + qk_plus_1_dot * delta_t
        qk = qk_plus_1
        
        # End-effector position
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        q1, q2, q3, q4, q5, q6 = qk
        o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
        # print(f"End Effector: {o_n_numerical}")

        # if (o_n_numerical[1] > -112 or o_n_numerical[2] > 1045):
        if (o_n_numerical[2] > 1048):
            print("Reached")
            return

        publish_values(qk)


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


def publish_values(qk, publishing_rate):
    rate = rospy.Rate(1)

    q1, q2, q3, q4, q5, q6 = qk

    pub_joint1_pos.publish(q1 + math.pi/2)
    pub_joint2_pos.publish(q2 + math.pi/2)
    pub_joint3_pos.publish(q3 + math.pi/2)
    pub_joint4_pos.publish(q4)
    pub_joint5_pos.publish(q5)
    pub_joint6_pos.publish(q6)
    rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('move_robot', anonymous=False)
        o_n, J = g05()

        pub_joint1_pos = rospy.Publisher('/g05/Revolute_1_position_controller/command', Float64, queue_size=10) 
        pub_joint2_pos = rospy.Publisher('/g05/Revolute_2_position_controller/command', Float64, queue_size=10)
        pub_joint3_pos = rospy.Publisher('/g05/Revolute_3_position_controller/command', Float64, queue_size=10)
        pub_joint4_pos = rospy.Publisher('/g05/Revolute_4_position_controller/command', Float64, queue_size=10)
        pub_joint5_pos = rospy.Publisher('/g05/Revolute_5_position_controller/command', Float64, queue_size=10)
        pub_joint6_pos = rospy.Publisher('/g05/Revolute_6_position_controller/command', Float64, queue_size=10)

        reset_robot()
        follow_trajectory(o_n, J)

    except rospy.ROSInterruptException: 
        pass