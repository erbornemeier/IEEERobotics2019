#!/usr/bin/env python

import rospy
import rospkg
import subprocess
import json
import re
import time
import Queue

from socket import *

from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Pose2D

rospy.init_node("visualizer")
server_ip = rospy.get_param("/visualizer/server_ip")
server_port = rospy.get_param("/visualizer/server_port")

client_socket = socket(AF_INET, SOCK_DGRAM)
client_socket.settimeout(1)
addr = (server_ip, int(server_port))

# isConnected = False
# attempts = 3
# while attempts > 0:
#     client_socket.sendto("initialize", addr)

#     try:
#         data, server = clientSocket.recvfrom(1024)
#         isConnected = true
#         print("[Visualizer] Connected to server")
#     except:
#         print("Not connected")
#         time.sleep(1)
#         attempts -= 1
    
# if not isConnected:
#     print("Continuing without visualizer")
#     exit()

client_socket.sendto("init-robot", addr)
msg_queue = Queue.Queue()

def sendCommand(msg):
    msg_queue.put(msg.data)
    # client_socket.sendto(msg.data, addr)

def sendPose(msg):
    client_socket.sendto("update-robot-pose x:{} y:{} theta:{}".format(msg.x, msg.y, msg.theta), addr)

pose_sub = rospy.Subscriber('robot_pose', Pose2D, sendPose)
command_sub = rospy.Subscriber('vis_command', String, sendCommand)

while not rospy.is_shutdown():
    if not msg_queue.empty():
        client_socket.sendto(msg_queue.get(), addr)
        time.sleep(0.1)
