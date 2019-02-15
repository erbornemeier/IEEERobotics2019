#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from state_machine.state_machine import StateMachine
from state_machine.drive_to_block_state import DriveToBlockState
import time as t

rospy.init_node("scheduler")

state_machine = StateMachine(DriveToBlockState())

_ = raw_input("Press enter to start")

while not rospy.is_shutdown():
    state_machine.run()
