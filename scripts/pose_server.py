#!/usr/bin/env python

"""
Pose server for iOS to ROS
"""

from handle_udp import extractUDP
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float64, Float64MultiArray, String, Int32
import sys
from tf.transformations import *
from math import pi
import numpy as np
import tf

ios_clock_valid = False
ios_clock_offset = -1.0

rospy.init_node("iOS_pose")
last_timestamp = rospy.Time.now()

port = rospy.get_param('~port_number')
pose_topic = rospy.get_param('~pose_topic')
coordinate_frame = rospy.get_param('~coordinate_frame')
pub_pose = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
pub_clock = None
pub_clock = rospy.Publisher('/ios_clock', Float64, queue_size=10)
br = tf.TransformBroadcaster()

def handle_pose():
    global ios_clock_valid
    global ios_clock_offset
    global last_timestamp

    pose_data = extractUDP(udp_port=35601)
    pose_vals = pose_data.split(",")
    position_vals = pose_vals[12:15]
    ios_timestamp = pose_vals[16]


    ROS_timestamp = rospy.Time.now()
    if not(ios_clock_valid):
        ios_clock_offset = ROS_timestamp.to_time() - float(ios_timestamp)
        ios_clock_valid = True

    pub_clock.publish(ios_clock_offset)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time(ios_clock_offset + float(ios_timestamp))
    msg.header.frame_id = coordinate_frame
    eulers = pose_vals[17:]
    # quat = quaternion_from_euler(float(eulers[0])*-1, float(eulers[1])*-1, float(eulers[2]), axes="sxyz")
    pose_vals = [float(x) for x in pose_vals[:16]]
    rotation_matrix = np.matrix([pose_vals[0:4], pose_vals[4:8], pose_vals[8:12], pose_vals[12:16]]).T

    # rotation_matrix = np.hstack((-rotation_matrix[:,2], -rotation_matrix[:,0], rotation_matrix[:,1], rotation_matrix[:,3]))
    # rotation_matrix = np.asarray(rotation_matrix)

    change_basis = np.matrix([[0,0,-1,0],[-1,0,0,0],[0,1,0,0],[0,0,0,1]])
    # change_basis.T
    change_basis2 = np.matrix([[0,0,-1,0],[0,-1,0,0],[-1,0,0,0],[0,0,0,1]])
    print rotation_matrix
    new_mat = change_basis*rotation_matrix*change_basis2
    print new_mat
    new_mat = new_mat.A
    print "test", type(new_mat)


    # rotation_mat = np.array([pose_vals[0:4], pose_vals[4:8], pose_vals[8:12], pose_vals[12:16]]).T
    # print(rotation_matrix)
    # eulers = [float(eulers[i]) for i in range(len(eulers))]
    # temp = eulers[0]
    # eulers[0] = eulers[1]
    # eulers[1] = temp
    #
    # new_mat = compose_matrix(angles=eulers, translate=[float(position_vals[i]) for i in range(len(position_vals))])
    # rotation_matrix[[0,1]] = rotation_matrix[[1,0]]
    #eulers = pose_vals[17:]
    trans = translation_from_matrix(new_mat)
    #new_mat = np.hstack((rotation_matrix[:,1], rotation_matrix[:,0], rotation_matrix[:,2], rotation_matrix[:,3]))
    quat = quaternion_from_matrix(new_mat)
    #
    # msg.pose.position.x = -float(position_vals[2])
    # msg.pose.position.y = float(position_vals[0])
    # msg.pose.position.z = float(position_vals[1])
    print trans, type(trans)

    msg.pose.position.x = trans[0]
    msg.pose.position.y = trans[1]
    msg.pose.position.z = trans[2]

    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]

    print msg


    br.sendTransform([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                     [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
                     msg.header.stamp,
                     "real_device",
                     "odom")





    pub_pose.publish(msg)

while True:
    handle_pose()
