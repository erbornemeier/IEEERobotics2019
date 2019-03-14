import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool

class ThrowBlockState(State):
    def __init__(self):
        super(ThrowBlockState, self).__init__("Throw Block")

    def start(self):
        rospy.loginfo("Throw block state start")

    def run(self):
        t.sleep(0.5)
        commands.send_grip_command(commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_grip_command(commands.CLAW_OPEN)
        commands.send_claw_command(70)
        rospy.loginfo("Picked up block")
        t.sleep(0.5)
        from detect_letter_state import * 
        return DetectLetterState()

    def finish(self):
        rospy.loginfo("Finished pick up block state")
