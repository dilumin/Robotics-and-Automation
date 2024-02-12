from geometry_msgs.msg import Quaternion
import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
# import rospy
# import tf.transformations as tf
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


def inverse_of_quaternion(q):
    q_con = [0,0,0,0]
    q_norm = np.linalg.norm(q)

    q_con[0] = q[0]/q_norm**2
    q_con[1] = (q[1] * -1)/q_norm**2
    q_con[2] = (q[2] * -1) /q_norm**2
    q_con[3] = (q[3] * -1)/q_norm**2
    return q_con


q1 =[ [1,2,3,4] ,[3 , 4 ,-5 , 6] , [4 , -5 , 6 ,-7] , [7,8,-9,10] , [8,-9,10,-11] , [10 , -11,12,-13] ]
q2 =[ [5 , 6 , 7 ,8] , [7,-8,9,-10] , [8,9,-10,11] , [11,-12,13,-14] , [12,13,-14,15],[14,15,-16,17]]
for i in range(6):
    q1_inv =  inverse_of_quaternion(q1[i])
    q_r = quaternion_multiply(q2[i] , inverse_of_quaternion(q1[i]))
    print("q1: " + str(q1[i][0])+ " + " +str(q1[i][1])+ "i + " + str(q1[i][2])+ "j + "+ str(q1[i][3])+"k ")
    print("q2: " + str(q2[i][0])+ " + " +str(q2[i][1])+ "i + " + str(q2[i][2])+ "j + "+ str(q2[i][3])+"k ")
    print("q1_inv: " + str(q1_inv[0])+ " + " +str(q1_inv[1])+ "i + " + str(q1_inv[2])+ "j + "+ str(q1_inv[3])+"k ")
    print("q_r: " + str(q_r[0])+ " + " +str(q_r[1])+ "i + " + str(q_r[2])+ "j + "+ str(q_r[3])+"k ")
    print("\n")




