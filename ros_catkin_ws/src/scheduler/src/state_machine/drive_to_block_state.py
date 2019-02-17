from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import time as t
import rospy

class DriveToBlockState(State):
    def __init__(self):
        super(DriveToBlockState, self).__init__("Drive to Block State")

    def get_block_pos_cb(self, msg):
        self.block_pos = (msg.x, msg.y)

    def start(self):
        rospy.loginfo("Entering drive to block state")
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.block_pos = (None, None)
        
        rospy.wait_for_service("block_pos")
        self.block_srv = rospy.ServiceProxy("block_pos", Block)
        
        t.sleep(1)
        self.cameraAngle = 20
        commands.send_cam_command(self.cam_pub, self.cameraAngle)
        t.sleep(1)
        rospy.loginfo("Set camera to starting angle 20")

        self.gain = 6 

    def run(self):

        # Coordinate system [0,1] top left corner is (0,0)
        try:
            block_pos = self.block_srv()
        except Exception as e:
            commands.send_drive_command(self.drive_pub, 0, 0, 0)
            self.cameraAngle = 20 
            commands.send_cam_command(self.cam_pub, int(self.cameraAngle))
            print(e)
            return self

        rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.cameraAngle))
    
        if block_pos.y < 0:
            commands.send_drive_command(self.drive_pub, 0, 0, 0)
            self.cameraAngle = 20 
            commands.send_cam_command(self.cam_pub, int(self.cameraAngle))
            return self


        if block_pos.y > 0.53:
            self.cameraAngle += self.gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.cameraAngle -= self.gain * (0.5 - block_pos.y) 

        if self.cameraAngle < 20:
            self.cameraAngle = 20
        elif self.cameraAngle > 47:
            self.cameraAngle = 47

        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))
        
        if block_pos.x < 0.3:
            commands.send_drive_command(self.drive_pub, 0, 0, 0.3)
        elif block_pos.x > 0.7:
            commands.send_drive_command(self.drive_pub, 0, 0, -0.3)
        else:
            commands.send_drive_command(self.drive_pub, 1, 0, 0)


        if self.cameraAngle == 47:
            t.sleep(0.2)
            commands.send_drive_command(self.drive_pub, 0, 0, 0)
            from pick_up_block_state import *
            return PickUpBlockState()
        else:
            return self
        
    def finish(self):
        self.drive_pub.unregister()
        self.cam_pub.unregister()
        self.block_srv.close()
        rospy.loginfo("Exiting drive to block state")

