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
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.claw_grip_pub = rospy.Publisher("grip_command", Bool, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
        rospy.loginfo("Throw block state start")

    def run(self):
        t.sleep(0.5)
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_CLOSED)
        rospy.Rate(2).sleep()
        commands.send_grip_command(self.claw_grip_pub, commands.CLAW_OPEN)
        commands.send_claw_command(self.claw_pub, 70)
        rospy.loginfo("Picked up block")
        t.sleep(0.5)
        from detect_letter_state import * 
        return DetectLetterState()

    def finish(self):
        self.claw_pub.unregister()
        self.cam_pub.unregister()
        self.display_letter_pub.unregister()
        rospy.loginfo("Finished pick up block state")
