import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import time as t
import math
import rospy

class DriveToBlockState(State):
    def __init__(self):
        super(DriveToBlockState, self).__init__("Drive to Block State")

    def start(self):
        rospy.loginfo("Entering drive to block state")

        self.block_x = (7.5-globals.x_coords[globals.current_block])*12
        self.block_y = (globals.y_coords[globals.current_block]+0.5)*12
        self.robot_x = -1
        self.robot_y = -1
        self.robot_theta = -1
        self.needs_approach = True

        self.claw_pub = rospy.Publisher("claw_command", UInt8, queue_size=1)
        self.drive_pub = rospy.Publisher("drive_command", Pose2D, queue_size=1)
        self.cam_pub = rospy.Publisher("cam_command", UInt8, queue_size=1)
        self.pose_sub = rospy.Subscriber("robot_pose", Pose2D, self.__set_pose__)
        rospy.wait_for_service("block_pos")
        self.block_srv = rospy.ServiceProxy("block_pos", Block)

        self.cam_gain = 6 
        self.drive_gain = 2/27.
        self.turn_gain = 2
        self.cameraAngle = 20
        self.rate = rospy.Rate(5)

        self.camera_target_angle = 47
        self.close_dist = 36 #inches

        commands.send_cam_command(self.cam_pub, self.cameraAngle)
        rospy.loginfo("Set camera to starting angle 20")

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def __get_close_to_block__(self):
        # wait until robot pos recieved
        while (self.robot_x == -1):
            print("waiting for pose")
            rospy.Rate(2).sleep()

        dx, dy = self.robot_x - self.block_x, self.robot_y - self.block_y
        forward_dist = (dx**2 + dy**2)**0.5 - self.close_dist 
        #flip dx since the axis is backwards
        turn_angle = 180 - ((math.atan2(dy, -dx) * 180.0/3.14159) - self.robot_theta)
        if turn_angle > 180:
            turn_angle -= 360
        if turn_angle < -180:
            turn_angle += 360
        print("TURNING: {} THEN DRIVING {}".format(turn_angle, forward_dist))
        commands.send_drive_turn_command(self.drive_pub, turn_angle)
        rospy.Rate(0.2).sleep()
        commands.send_drive_forward_command(self.drive_pub, forward_dist)
        rospy.Rate(0.15).sleep()

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
        commands.send_drive_vel_command(self.drive_pub, 0, 0)

    def __camera_to_block__(self, block_pos):

        if block_pos.y > 0.53:
            self.cameraAngle += self.cam_gain * (block_pos.y - 0.5)
        elif block_pos.y < 0.47:
            self.cameraAngle -= self.cam_gain * (0.5 - block_pos.y) 

        if self.cameraAngle < 20:
            self.cameraAngle = 20
        elif self.cameraAngle > self.camera_target_angle:
            self.cameraAngle = self.camera_target_angle

        commands.send_cam_command(self.cam_pub, int(self.cameraAngle))

    def __drive_to_block__(self, block_pos):

        turn_speed = self.turn_gain * (0.5 - block_pos.x)
        forward_speed = self.drive_gain * (self.camera_target_angle - self.cameraAngle) + 0.2
        commands.send_drive_vel_command(self.drive_pub, forward_speed, turn_speed)


    def run(self):

        self.rate.sleep()

        commands.send_claw_command(self.claw_pub, commands.DROP_ANGLE)
        if self.needs_approach:
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

        rospy.loginfo("Block Pos: " + str(block_pos.x) + ", " + str(block_pos.y) + " Cam Angle: " + str(self.cameraAngle))

        if self.cameraAngle == self.camera_target_angle:
            commands.send_drive_vel_command(self.drive_pub, 0, 0)
            from pick_up_block_state import *
            return PickUpBlockState()
        else:
            return self
        
    def finish(self):
        self.drive_pub.unregister()
        self.cam_pub.unregister()
        self.block_srv.close()
        rospy.loginfo("Exiting drive to block state")

