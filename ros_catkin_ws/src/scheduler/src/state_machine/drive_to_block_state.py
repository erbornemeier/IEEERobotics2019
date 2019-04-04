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

        #self.block_pos = ( (globals.x_coords[globals.current_block])*12 + 6,\
        #                   (globals.y_coords[globals.current_block])*12 + 6)
        next_block = globals.block_queue[0]
        self.block_pos = ( (next_block[0])*12 + 6,\
                           (next_block[1])*12 + 6 )
        self.needs_approach = True

        self.cam_gain = 9 
        self.drive_gain = 2/27.
        self.min_speed = 0.5
        self.turn_gain = 4
        self.rate = rospy.Rate(5)

        self.camera_start_angle = 20
        self.camera_target_angle = 48 
        self.switch_point = 30 
        self.approach_dist = 18 #inches 

        commands.set_display_state(commands.NORMAL)

        self.camera_angle = self.camera_start_angle

        self.SEEN_TIMEOUT = 2
        self.last_seen = t.time()


    def __get_block_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.block_srv()
        except Exception as e:
            #print(e)
            block_pos = BlockResponse()
            block_pos.x = -1
            block_pos.y = -1
            return block_pos

    def __get_block_close_pos__(self):
        # Coordinate system [0,1] top left corner is (0,0)
        try:
            return commands.block_srv_close()
        except Exception as e:
            #print(e)
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
        forward_speed = self.drive_gain * (self.camera_target_angle - self.camera_angle) + self.min_speed 
        commands.send_drive_vel_command(forward_speed, turn_speed)


    def run(self):

        self.rate.sleep()

        if self.needs_approach:

            commands.send_cam_command(self.camera_angle)
            commands.send_grip_command(commands.CLAW_CLOSED)
            commands.send_claw_command(commands.CARRY_ANGLE)

            success = drive_utils.go_to_point(self.block_pos, self.approach_dist)
            if not success:
                #move block to back of queue
                globals.block_queue.append(globals.block_queue.popleft())
                return DriveToBlockState()

            turn_angle, _ = drive_utils.get_drive_instructions(self.block_pos)
            #print("TURNING TO FACE BLOCK: {}".format(turn_angle))
            drive_utils.turn(turn_angle)
            self.needs_approach = False
            self.last_seen = t.time()
        
        commands.send_grip_command(commands.CLAW_OPEN)
        #camera to block
        if self.camera_angle < self.switch_point:
            block_pos = self.__get_block_pos__()
        else:
            block_pos = self.__get_block_close_pos__()

        if block_pos.y < 0:
            if t.time() - self.last_seen > self.SEEN_TIMEOUT:
                drive_utils.drive(-2)
                self.camera_angle -= 1
                commands.send_cam_command(int(self.camera_angle))
                self.last_seen = t.time()
            else:
                self.__reset__()
            return self
        else:
            self.last_seen = t.time()
            self.__camera_to_block__(block_pos)
            self.__drive_to_block__(block_pos)

        #rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.camera_angle))

        if self.camera_angle == self.camera_target_angle:
            t.sleep(0.25)
            commands.send_drive_vel_command(0, 0)
            from pick_up_block_state import PickUpBlockState 
            return PickUpBlockState()
        else:
            return self
