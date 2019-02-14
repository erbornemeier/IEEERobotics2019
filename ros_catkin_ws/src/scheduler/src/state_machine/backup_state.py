from state import State
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
import commands
import time as t
import rospy

class BackupState(State):
    def __init__(self):
        super(BackupState, self).__init__("Backup State")

    def start(self):
        rospy.loginfo("Entering backup state")
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)

    def run(self):
        #TODO: incorporate actual block data
        commands.send_drive_command(self.drive_pub, -6, 0, 0 )
        t.sleep(8)

        from drive_to_block_state import * 
        return DriveToBlockState()

    def finish(self):
        self.drive_pub.unregister()
        rospy.loginfo("Exiting backup state")
