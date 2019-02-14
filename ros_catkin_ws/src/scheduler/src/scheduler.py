#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from pick_up_block_state import PickUpBlockState
import time as t

drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)

display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
display_robot_pos_pub = rospy.Publisher("display_robot_pos", Pose2D, queue_size = 1)

current_letter = 0
def recieve_letter(msg):
    global current_letter
    current_letter = msg.data

rospy.Subscriber("letter_identifier", UInt8, recieve_letter)

rospy.init_node("scheduler")

state_machine = StateMachine(PickUpBlockState())

# def send_drive_command(x, y, theta):
#     msg = Pose2D()
#     msg.x = x
#     msg.y = y
#     msg.theta = theta
#     drive_pub.publish(msg)
#
# PICKUP = 0
# PUTDOWN = 1
# def send_claw_command(cmd):
#     msg = UInt8()
#     msg.data = cmd
#     claw_pub.publish(msg)
#
# def send_cam_command(angle):
#     msg = UInt8()
#     msg.data = angle
#     cam_pub.publish(msg)
#
# def display_letter(letter):
#     msg = UInt8()
#     msg.data = letter
#     print("Sending: " + str(msg.data))
#     display_letter_pub.publish(msg)
#
# def display_pos(pose):
#     msg = Pose2D()
#     msg.x = pose.x
#     msg.y = pose.y
#     display_robot_pos_pub.publish(msg)


while not rospy.is_shutdown():
    _ = raw_input("Press enter to start")
    state_machine.run()
    # send_drive_command(0,6,0)
    # t.sleep(5)
    # send_claw_command(PICKUP)
    # send_cam_command(37)
    # display_letter(0xFF)
    # t.sleep(8)
    # display_letter(current_letter)
    # send_claw_command(PUTDOWN)
