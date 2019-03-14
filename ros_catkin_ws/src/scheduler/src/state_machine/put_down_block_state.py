import globals
from state import State
from std_msgs.msg import UInt8, Bool
from object_detection.srv import *
import commands
import rospy

class PutDownBlockState(State):
    def __init__(self):
        super(PutDownBlockState, self).__init__("Put Down Block")

    def start(self):
        rospy.loginfo("Entering put down block state")

    def run(self):
        commands.send_cam_command(0)
        commands.send_grip_command(commands.CLAW_OPEN)
        commands.send_claw_command(commands.DROP_ANGLE)
        from backup_state import * 
        return BackupState()

    def finish(self):
        rospy.loginfo("Exiting put down block state")

