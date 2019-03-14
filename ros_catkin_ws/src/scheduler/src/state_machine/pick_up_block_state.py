import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool

class PickUpBlockState(State):
    def __init__(self):
        super(PickUpBlockState, self).__init__("Pick Up Block")

    def start(self):
        rospy.loginfo("Pick up block state start")
        self.cam_pickup_angle = 40

    def run(self):
        commands.send_cam_command(self.cam_pickup_angle) 
        rospy.Rate(2).sleep()
        commands.send_grip_command(commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_claw_command(commands.PICKUP_ANGLE)
        rospy.loginfo("Picked up block")
        rospy.Rate(2).sleep()
        from detect_letter_state import * 
        return DetectLetterState()

    def finish(self):
        rospy.loginfo("Finished pick up block state")
