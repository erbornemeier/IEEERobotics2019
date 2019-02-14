#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from state_machine.drive_to_block_state import DriveToBlockState
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

state_machine = StateMachine(DriveToBlockState())

_ = raw_input("Press enter to start")

while not rospy.is_shutdown():
    state_machine.run()
