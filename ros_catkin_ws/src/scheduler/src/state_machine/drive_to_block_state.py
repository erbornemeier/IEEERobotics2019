import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import drive_utils
import time as t
import math
import rospy

class DriveToBlockState(State):
    def __init__(self):
        super(DriveToBlockState, self).__init__("Drive to Block State")

    def start(self):
        super(DriveToBlockState, self).start()

        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)

        self.block_x = (globals.x_coords[globals.current_block]+0.5)*12
        self.block_y = (globals.y_coords[globals.current_block]+0.5)*12
        self.robot_x = -1
        self.robot_y = -1
        self.robot_theta = -1
        self.needs_approach = True

        self.cam_gain = 6 
        self.drive_gain = 1.5/27.
        self.turn_gain = 3
        self.cameraAngle = 20
        self.rate = rospy.Rate(5)

        self.camera_target_angle = 50 
        self.approach_dist = 18 #inches

        t.sleep(0.5)
        commands.set_display_state(commands.NORMAL)

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def __get_close_to_block__(self):
        # wait until robot pos recieved
        t.sleep(1)
        int_point = drive_utils.drive_safely((self.block_x, self.block_y), self.approach_dist) 
        print("INTERMEDIATE POINT: {}".format(int_point))
        if int_point is not None:
            drive_utils.go_to_point(int_point)
        
        print("TARGET POINT: {}".format((self.block_x, self.block_y)))
        drive_utils.go_to_point((self.block_x, self.block_y), self.approach_dist)


    def __get_block_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.block_srv()
        except Exception as e:
            print(e)
            block_pos = BlockResponse()
            block_pos.x = -1
            block_pos.y = -1
            return block_pos

    def __reset__(self):
        #self.cameraAngle = 20
        commands.send_cam_command(int(self.cameraAngle))
        commands.send_drive_vel_command(0, 0)

    def __camera_to_block__(self, block_pos):

        if block_pos.y > 0.53:
            self.cameraAngle += self.cam_gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.cameraAngle -= self.cam_gain * (0.5 - block_pos.y) 

        if self.cameraAngle < 20:
            self.cameraAngle = 20
        elif self.cameraAngle > self.camera_target_angle:
            self.cameraAngle = self.camera_target_angle

        commands.send_cam_command(int(self.cameraAngle))

    def __drive_to_block__(self, block_pos):

        turn_speed = self.turn_gain * (0.5 - block_pos.x)
        forward_speed = self.drive_gain * (self.camera_target_angle - self.cameraAngle) + 0.2
        commands.send_drive_vel_command(forward_speed, turn_speed)


    def run(self):

        self.rate.sleep()

        if self.needs_approach:
            commands.send_cam_command(self.cameraAngle)
            commands.send_claw_command(commands.DROP_ANGLE)
            self.__get_close_to_block__()
            self.needs_approach = False

        #camera to block
        block_pos = self.__get_block_pos__()
        if block_pos.y < 0:
            self.__reset__()
            return self
        else:
            self.__camera_to_block__(block_pos)

        self.__drive_to_block__(block_pos)

        #rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.cameraAngle))

        if self.cameraAngle == self.camera_target_angle:
            t.sleep(0.25)
            commands.send_drive_vel_command(0, 0)
            from pick_up_block_state import *
            return PickUpBlockState()
        else:
            return self
