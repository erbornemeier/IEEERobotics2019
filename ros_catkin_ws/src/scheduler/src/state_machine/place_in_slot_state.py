import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D

class PlaceInSlotState(State):
    def __init__(self):
        super(PlaceInSlotState, self).__init__("Place in Slot")

    def start(self):
        rospy.loginfo("Place in slot state start")

        self.forward_dist = 3 #inches
        self.side_angle = 0
        if globals.current_letter % 3 == 0:
            self.side_angle = 20
        elif globals.current_letter % 3 == 2:
            self.side_angle = -20

    def __drop_off_at_letter__(self):
        #drive to slot
        if self.side_angle != 0:
            print('SENDING TURN: {}'.format(self.side_angle))
            commands.send_drive_turn_command(self.side_angle)
            rospy.Rate(0.25)
        commands.send_drive_forward_command(self.forward_dist)
        rospy.Rate(0.25).sleep()

        #drop in slot
        commands.send_grip_command(commands.CLAW_OPEN)
        rospy.Rate(1).sleep()

        #back off slot
        commands.send_drive_forward_command(-self.forward_dist)
        rospy.Rate(0.25).sleep()
        if self.side_angle != 0:
            print('SENDING TURN: {}'.format(-self.side_angle))
            commands.send_drive_turn_command(-self.side_angle)
            rospy.Rate(0.25)

        globals.current_block += 1

    def run(self):
        #close claw and lift it up
        rospy.Rate(2).sleep()
        commands.send_grip_command(commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_claw_command(commands.PICKUP_ANGLE)
        rospy.Rate(2).sleep()

        #drop it off
        self.__drop_off_at_letter__()
        rospy.Rate(2).sleep()

        from drive_to_block_state import * 
        return DriveToBlockState()

    def finish(self):
        rospy.loginfo("Finished pick up block state")
