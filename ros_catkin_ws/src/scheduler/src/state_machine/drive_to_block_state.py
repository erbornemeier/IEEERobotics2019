from state import State
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
import commands
import time as t
import rospy

class DriveToBlockState(State):
    def __init__(self):
        super(DriveToBlockState, self).__init__("Drive to Block State")

    def start(self):
        rospy.loginfo("Entering drive to block state")
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.block_pos_sub = rospy.Subscriber("block_pos", Pose2D, queue_size=1)

    def run(self):
        #TODO: incorporate actual block data
        commands.send_drive_command(self.drive_pub, 6, 0, 180 )
        t.sleep(8)

        from pick_up_block_state import * 
        return PickUpBlockState()

    def finish(self):
        self.drive_pub.unregister()
        self.cam_pub.unregister()
        self.block_pos_sub.unregister()
        rospy.loginfo("Exiting drive to block state")
