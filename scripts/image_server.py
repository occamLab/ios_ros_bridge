#!/usr/bin/env python

from handle_udp import extractUDP
import rospy
import time
import socket
import struct
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage

ios_clock_offset = 0

def handle_ios_clock(msg):
    global ios_clock_offset
    ios_clock_offset = msg.data

#rospy to interface with ROS setup
rospy.init_node("image_server")

host = ''
port = rospy.get_param('~port_number')
camera_name = rospy.get_param('~camera_name')

clock_sub = rospy.Subscriber('/ios_clock', Float64, handle_ios_clock)
pub_camera = rospy.Publisher('/' + camera_name + '/image_raw/compressed', CompressedImage, queue_size=10)

image_data = {}
msg = CompressedImage()

UDP_IP = "0.0.0.0"
UDP_PORT = 35602

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def handle_image():
    data, addr = sock.recvfrom(1600)
    image_number, packet_number = struct.unpack('<BB', data[0:2])
    if packet_number == 0:
        total_packets = struct.unpack('<B', data[2])[0]
        image_data[image_number] = {}
        image_data[image_number]['packets_expected'] = total_packets
        image_data[image_number]['packets_received'] = 1
        time_bytes = struct.unpack('<B', data[3])[0]
        stampedTime = data[4:4+time_bytes]
        image_data[image_number]['timestamp'] = stampedTime
        image_data[image_number]['payload'] = [(packet_number, data[4+time_bytes:])]

    elif image_number in image_data.keys():
        image_data[image_number]['packets_received'] += 1
        image_data[image_number]['payload'] += [(packet_number, data[2:])]

        if image_data[image_number]['packets_received'] == image_data[image_number]['packets_expected']:
            image_data[image_number]['payload'].sort()
            image = ''
            for packet in image_data[image_number]['payload']:
                image += packet[1]
            msg.data = image
            msg.header.stamp  = rospy.Time.now()#rospy.Time(ios_clock_offset + float(ts))
            msg.header.frame_id = camera_name
            msg.data = image
            msg.format = 'jpeg'
            pub_camera.publish(msg)

while True:
    handle_image()
