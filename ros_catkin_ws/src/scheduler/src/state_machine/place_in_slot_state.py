import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D

class PlaceInSlotState(State):
    def __init__(self):
        super(PlaceInSlotState, self).__init__("Throw Block")

    def start(self):
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.claw_grip_pub = rospy.Publisher("grip_command", Bool, queue_size=1)
        rospy.loginfo("Place in slot state start")

        self.forward_dist = 3 #inches
        self.side_angle = 30

    def __drop_off_at_letter__(self):
        #a or d
        turn_angle = 0
        if globals.current_letter % 3 == 0:
            turn_angle = self.side_angle
        #c or f
        elif globals.current_letter % 3 == 2:
            turn_speed = -self.side_angle

        #drive to slot
        if turn_angle != 0:
            command.send_drive_turn_command(self.drive_pub, turn_angle )
            rospy.Rate(0.5)
        commands.send_drive_forward_command(self.drive_pub, self.forward_dist)
        rospy.Rate(0.5).sleep()

        #drop in slot
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_OPEN)
        rospy.Rate(1).sleep()

        #back off slot
        commands.send_drive_forward_command(self.drive_pub, -self.forward_dist)
        rospy.Rate(0.5).sleep()
        if turn_angle != 0:
            command.send_drive_turn_command(self.drive_pub, -turn_angle)
            rospy.Rate(0.5)

    def run(self):
        #close claw and lift it up
        rospy.Rate(2).sleep()
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_claw_command(self.claw_pub, commands.PICKUP_ANGLE)
        rospy.Rate(2).sleep()

        #drop it off
        self.__drop_off_at_letter__()
        rospy.Rate(2).sleep()

        from drive_to_block_state import * 
        return DriveToBlockState()

    def finish(self):
        self.claw_pub.unregister()
        self.cam_pub.unregister()
        self.claw_grip_pub.unregister()
        self.display_letter_pub.unregister()
        rospy.loginfo("Finished pick up block state")
