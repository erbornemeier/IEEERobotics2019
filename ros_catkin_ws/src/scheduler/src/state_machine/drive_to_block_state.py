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

        self.block_pos = ( (globals.x_coords[globals.current_block]+0.5)*12,\
                           (globals.y_coords[globals.current_block]+0.5)*12 )
        self.needs_approach = True

        self.cam_gain = 6 
        self.drive_gain = 1.5/27.
        self.turn_gain = 3
        self.rate = rospy.Rate(5)

        self.camera_start_angle = 20
        self.camera_target_angle = 50 
        self.approach_dist = 18 #inches

        commands.set_display_state(commands.NORMAL)

        self.camera_angle = self.camera_start_angle


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
        #self.camera_angle = 20
        commands.send_cam_command(int(self.camera_angle))
        commands.send_drive_vel_command(0, 0)

    def __camera_to_block__(self, block_pos):

        if block_pos.y > 0.53:
            self.camera_angle += self.cam_gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.camera_angle -= self.cam_gain * (0.5 - block_pos.y) 

        if self.camera_angle < self.camera_start_angle:
            self.camera_angle = self.camera_start_angle
        elif self.camera_angle > self.camera_target_angle:
            self.camera_angle = self.camera_target_angle

        commands.send_cam_command(int(self.camera_angle))

    def __drive_to_block__(self, block_pos):

        turn_speed = self.turn_gain * (0.5 - block_pos.x)
        forward_speed = self.drive_gain * (self.camera_target_angle - self.camera_angle) + 0.2
        commands.send_drive_vel_command(forward_speed, turn_speed)


    def run(self):

        self.rate.sleep()

        if self.needs_approach:
            commands.send_cam_command(self.camera_angle)
            commands.send_claw_command(commands.DROP_ANGLE)
            #drive_utils.go_to_point(self.block_pos, self.approach_dist)
            drive_utils.go_to_point(self.block_pos)
            drive_utils.wait_for_pose_update()
            turn_angle, _ = drive_utils.get_drive_instructions(self.block_pos)
            print("TURNING TO FACE BLOCK: {}".format(turn_angle))
            drive_utils.turn(turn_angle)

            self.needs_approach = False

        #camera to block
        block_pos = self.__get_block_pos__()
        if block_pos.y < 0:
            self.__reset__()
            return self
        else:
            self.__camera_to_block__(block_pos)
            self.__drive_to_block__(block_pos)

        #rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.camera_angle))

        if self.camera_angle == self.camera_target_angle:
            commands.send_drive_vel_command(0, 0)
            from pick_up_block_state import PickUpBlockState 
            return PickUpBlockState()
        else:
            return self
