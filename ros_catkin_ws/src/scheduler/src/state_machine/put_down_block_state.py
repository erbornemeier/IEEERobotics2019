from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy

class PutDownBlockState(State):
    def __init__(self):
        super(PutDownBlockState, self).__init__("Put Down Block")

    def start(self):
        rospy.loginfo("Entering put down block state")
        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)

    def run(self):
        commands.send_cam_command(self.cam_pub, 0)
        commands.send_claw_command(self.claw_pub, commands.PUTDOWN)
        from backup_state import * 
        return BackupState()

    def finish(self):
        self.claw_pub.unregister()
        self.cam_pub.unregister()
        rospy.loginfo("Exiting put down block state")

