#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
from sympy import symbols, cos, sin, pi, Matrix
import numpy as np
import math
import matplotlib.pyplot as plt

def getTransformationMatrix(theta, d, a, alpha):
    """
    Calculate the homogeneous transformation matrix using DH parameters.

    Parameters:
        theta: Joint angle (rotation about z-axis).
        d: Link offset (translation along z-axis).
        a: Link length (translation along x-axis).
        alpha : Link twist (rotation about x-axis).

    Returns:
        Matrix: Homogeneous transformation matrix.
    """
    # Define transformation matrices
    R_z_theta = Matrix([[cos(theta), -sin(theta), 0, 0],
                 [sin(theta), cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    T_z_d = Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, d],
                 [0, 0, 0, 1]])

    T_x_a = Matrix([[1, 0, 0, a],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    R_x_alpha = Matrix([[1, 0, 0, 0],
                 [0, cos(alpha), -sin(alpha), 0],
                 [0, sin(alpha), cos(alpha), 0],
                 [0, 0, 0, 1]])

    # Calculate the homogeneous transformation matrix
    T = R_z_theta * T_z_d * T_x_a * R_x_alpha

    return T


def getJacobianMatrix(T, o_n):
    """
    Calculate the Jacobian matrix.

    Parameters:
        T: List of transformation matrices
        o_n: co-ordinates of end-effector

    Returns:
        Matrix: Jacobian column.
    """
    ## J_1 caculation
    # Unit vector along z-axis
    Z_0 = Matrix([[0], [0], [1]])
    Jv_1 = Z_0.cross(o_n)
    Jw_1 = Z_0
    # Ji = [Jvi,
    #       Jwi]
    J_1 = Matrix.vstack(Jv_1, Jw_1)

    # List to store Jacobian columns J_1 to J_6
    J = [J_1]

    ## Calculating remaing Jacobian columns from 2 to 6
    # Ji needs Ti-1 which is stored at T[i-2]
    # Ji is stored in J[i-1]
    # Ex: J_2 = calculateJacobianColumn(T[0], o_n). T[0] stores T_1
    for i in range(2, len(T)+1):
        J_i = calculateJacobianColumn(T[i - 2], o_n)
        J.append(J_i)

    # Combine Jacobian columns to make Jacobian matrix
    J_matrix = J[0]
    for i in range(1, len(T)):
        J_matrix = Matrix.hstack(J_matrix, J[i])
    return J_matrix


def calculateJacobianColumn(T, o_n):
    """
    Calculate the Jacobian for ith column.

    Parameters:
        T: Transformation matrix
        o_n: co-ordinates of end-effector

    Returns:
        Matrix: Jacobian column.
    """
    # Co-ordinates matrix o i-1
    o_i_1 = T[:3, -1]

    # Rotational matrix R i-1
    R_i_1 = T[:3, :3]

    # Unit vector along z-axis
    k = Matrix([[0], [0], [1]])

    # Z i-1
    Zi_1 = R_i_1 * k

    Jvi = Zi_1.cross(o_n - o_i_1)
    Jwi = Zi_1

    # Ji = [Jvi,
    #       Jwi]
    Ji = Matrix.vstack(Jvi, Jwi)

    return Ji


def velocityMatrix():
    """
    Calculate the x_dot matrix for myCobot as per calculation given in pdf report

    Parameters: theta (polar parameter)

    Returns:
        Matrix: x_dot matrix (with respect to base frame)
    """
    # Check report for calculation (80 cm radius cirles)
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

def g05():
    """
    Calculate the end-effector position & Jacobian

    Parameters: None

    Returns:
        End effector position & Jacobian Matrix
    """

    """
        Symbolic calculations
    """
    # Define symbolic variables for joint angles (thetas) and link parameters (a, d) & alphas as constants
    theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
    a1, a2, a3, a4, a5, a6 = symbols('a1 a2 a3 a4 a5 a6')
    d1, d2, d3, d4, d5, d6 = symbols('d1 d2 d3 d4 d5 d6')

    # From DH table
    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = pi/2, pi, pi/2, -pi/2, pi/2, 0

    A_1 = getTransformationMatrix(theta1, d1, a1, alpha1)
    A_2 = getTransformationMatrix(theta2, d2, a2, alpha2)
    A_3 = getTransformationMatrix(theta3, d3, a3, alpha3)
    A_4 = getTransformationMatrix(theta4, d4, a4, alpha4)
    A_5 = getTransformationMatrix(theta5, d5, a5, alpha5)
    A_6 = getTransformationMatrix(theta6, d6, a6, alpha6)

    T = [A_1, A_1 * A_2, A_1 * A_2 * A_3, A_1 * A_2 * A_3 * A_4, A_1 * A_2 * A_3 * A_4 * A_5, A_1 * A_2 * A_3 * A_4 * A_5 * A_6]

    # Final transformation matrix
    T_6 = T[5]
    # End-effector co-ordinates matrix
    o_n = T_6[:3, -1]

    # Calculate Jacobian matrix
    J = getJacobianMatrix(T, o_n)

    """
        Numerical calculations

        From DH table:
        a1 = 0, a2 = -290, a3 = 0, a4 = 0, a5 = 0, a6 = 0

        d1 = 376.07, d2 = 138, d3 = -138, d4 = 333.93, d5 = 0, d6 = 60
    """

    o_n = o_n.subs([(a1, 0), (a2, -290), (a3, 0), (a4, 0), (a5, 0), (a6, 0),
                (d1, 426.07), (d2, 138), (d3, 138), (d4, 333.93), (d5, 0), (d6, 60)])

    J = J.subs([(a1, 0), (a2, -290), (a3, 0), (a4, 0), (a5, 0), (a6, 0),
                (d1, 426.07), (d2, 138), (d3, 138), (d4, 333.93), (d5, 0), (d6, 60)])
    return o_n, J

def publish_values(qk):
    # print(qk)

    rate = rospy.Rate(10)

    q1, q2, q3, q4, q5, q6 = qk

    pub_joint1_pos.publish(q1+ math.pi/2)
    pub_joint2_pos.publish(q2+math.pi/2)
    pub_joint3_pos.publish(q3+math.pi/2)
    pub_joint4_pos.publish(q4)
    pub_joint5_pos.publish(q5)
    pub_joint6_pos.publish(q6)
    rate.sleep()
    # print("Enter to continue...")
    # input()


def move_to_points(o_n):

    rate = rospy.Rate(1)
    
    # while (not rospy.is_shutdown()):

    for q in np.arange(-1.57, 0, 0.1):
        pub_joint2_pos.publish(q)
        pub_joint3_pos.publish(q)
        rate.sleep()
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        q1, q2, q3, q4, q5, q6 = [-math.pi/2, q - math.pi/2, q - math.pi/2, 0, math.pi/2,0]
        o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
        print(o_n_numerical)

    pub_joint4_pos.publish(math.pi)
    print("Rotated")

    for q in np.arange(0, 1.57, 0.1):
        pub_joint2_pos.publish(q)
        pub_joint3_pos.publish(q)
        rate.sleep()
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        q1, q2, q3, q4, q5, q6 = [-math.pi/2, q - math.pi/2, q - math.pi/2, math.pi, math.pi/2,0]
        o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
        print(o_n_numerical)
    
    print("Done")

def draw_line(o_n, J):
    q0 = Matrix([[-math.pi/2], [-3*math.pi/4], [-3*math.pi/4], [0], [math.pi/2], [0]])
    qk = q0
    num_steps = 100
    time_taken = 5
    step_size = 2*math.pi / num_steps
    delta_t = time_taken / num_steps

    for i in range(num_steps + 1):
        qk_plus_1_dot = jointVelocityMatrix(J, qk)

        qk_plus_1 = qk + qk_plus_1_dot * delta_t
        qk = qk_plus_1
        
        # End-effector position
        theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
        q1, q2, q3, q4, q5, q6 = qk
        o_n_numerical = o_n.subs([(theta1, q1), (theta2, q2), (theta3, q3), (theta4, q4), (theta5, q5), (theta6, q6)])
        print(f"End Effector: {o_n_numerical}")

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
    print("Enter to continue...")
    input()

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

        # draw_line(o_n, J)
        reset_robot()
        move_to_points(o_n)

    except rospy.ROSInterruptException: 
        pass