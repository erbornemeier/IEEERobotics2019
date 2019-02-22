import globals
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

    def start(self):
        rospy.loginfo("Entering drive to block state")

        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        rospy.wait_for_service("block_pos")
        self.block_srv = rospy.ServiceProxy("block_pos", Block)

        self.cam_gain = 6 
        self.drive_gain = 2/27.
        self.turn_gain = 2
        self.cameraAngle = 20
        self.rate = rospy.Rate(5)

        commands.send_cam_command(self.cam_pub, self.cameraAngle)
        rospy.loginfo("Set camera to starting angle 20")

    def __get_block_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return self.block_srv()
        except Exception as e:
            print(e)
            block_pos = BlockResponse()
            block_pos.x = -1
            block_pos.y = -1
            return block_pos

    def __reset__(self):
        #self.cameraAngle = 20
        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))
        commands.send_drive_command(self.drive_pub, 0, 0, 0)

    def __camera_to_block__(self, block_pos):

        if block_pos.y > 0.53:
            self.cameraAngle += self.cam_gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.cameraAngle -= self.cam_gain * (0.5 - block_pos.y) 

        if self.cameraAngle < 20:
            self.cameraAngle = 20
        elif self.cameraAngle > 47:
            self.cameraAngle = 47

        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))

    def __drive_to_block__(self, block_pos):

        '''
        if block_pos.x < 0.4:
            turn_speed = self.turn_gain * (0.5 - block_pos.x)
            commands.send_drive_command(self.drive_pub, 0, 0, turn_speed)
        elif block_pos.x > 0.6:
            turn_speed = self.turn_gain * (block_pos.x - 0.5)
            commands.send_drive_command(self.drive_pub, 0, 0, -turn_speed)
        else:
            forward_speed = self.drive_gain * (47 - self.cameraAngle)
            commands.send_drive_command(self.drive_pub, forward_speed, 0, 0)
        '''
        turn_speed = self.turn_gain * (0.5 - block_pos.x)
        forward_speed = self.drive_gain * (47 - self.cameraAngle) + 0.2
        commands.send_drive_command(self.drive_pub, forward_speed, 0, turn_speed)


    def run(self):

        self.rate.sleep()

        commands.send_claw_command(self.claw_pub, commands.DROP_ANGLE)


        #camera to block
        block_pos = self.__get_block_pos__()
        if block_pos.y < 0:
            self.__reset__()
            return self
        else:
            self.__camera_to_block__(block_pos)

        self.__drive_to_block__(block_pos)

        rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.cameraAngle))

        if self.cameraAngle == 47:
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

