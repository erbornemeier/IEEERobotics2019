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

    def __drop_off_at_letter__(self):
        #a or d
        turn_speed = 0
        if globals.current_letter % 3 == 0:
            turn_speed = 0.4
        elif globals.current_letter % 3 == 2:
            turn_speed = -0.4

        commands.send_drive_command(self.drive_pub, 1.1, 0, turn_speed)
        rospy.Rate(0.5).sleep()
        commands.send_drive_command(self.drive_pub, 0, 0, 0)
        rospy.Rate(0.5).sleep()
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_OPEN)
        rospy.Rate(1).sleep()
        commands.send_drive_command(self.drive_pub, -1.1, 0, -turn_speed)
        rospy.Rate(0.5).sleep()
        commands.send_drive_command(self.drive_pub, 0, 0, 0)
        rospy.Rate(2).sleep()
   

    def run(self):
        rospy.Rate(2).sleep()
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_claw_command(self.claw_pub, commands.PICKUP_ANGLE)
        rospy.Rate(2).sleep()
        self.__drop_off_at_letter__()
        t.sleep(0.5)
        from drive_to_block_state import * 
        return DriveToBlockState()

    def finish(self):
        self.claw_pub.unregister()
        self.cam_pub.unregister()
        self.claw_grip_pub.unregister()
        self.display_letter_pub.unregister()
        rospy.loginfo("Finished pick up block state")
