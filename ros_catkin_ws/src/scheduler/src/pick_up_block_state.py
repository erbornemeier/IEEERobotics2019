from state_machine.state import State
from detect_letter_state import DetectLetterState
import commands
import time as t
import rospy
from std_msgs.msg import UInt8

class PickUpBlockState(State):
    def __init__(self):
        super(PickUpBlockState, self).__init__("Pick Up Block")

    def start(self):
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.display_letter_pub = rospy.Publisher("display_letter", UInt8, queue_size = 1)
        rospy.loginfo("Starting test state")

    def run(self):
        commands.send_claw_command(self.claw_pub, commands.PICKUP)
        commands.send_cam_command(self.cam_pub, 37)
        commands.display_letter(self.display_letter_pub, 0xFF)
        t.sleep(1)
        rospy.loginfo("Picked up block")
        return DetectLetterState()

    def finish(self):
        self.claw_pub.unregister()
        self.cam_pub.unregister()
        self.display_letter_pub.unregister()
        rospy.loginfo("Finishing test state")
