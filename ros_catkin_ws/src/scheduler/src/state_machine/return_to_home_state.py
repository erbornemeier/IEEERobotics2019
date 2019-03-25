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

class ReturnToHomeState(State):
    def __init__(self):
        super(ReturnToHomeState, self).__init__("Return To Home State")

    def start(self):
        super(ReturnToHomeState, self).start()
        commands.set_display_state(commands.NORMAL)
        self.home_x = 54 
        self.home_y = 54
        self.home_theta = 90

        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)

        self.robot_x = -1
        self.robot_y = -1
        self.robot_theta = -1

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta


    def run(self):
        # wait until robot pos recieved
        t.sleep(1)
        int_point = drive_utils.drive_safely((self.home_x, self.home_y)) 
        print("INTERMEDIATE POINT: {}".format(int_point))
        if int_point is not None:
            drive_utils.go_to_point(int_point)
        
        print("TARGET POINT: {}".format((self.home_x, self.home_y)))
        drive_utils.go_to_point((self.home_x, self.home_y))

        t.sleep(1)
        self.robot_x = -1
        while (self.robot_x == -1):
            print("waiting for pose")
            t.sleep(0.5)
        
        turn_angle = self.home_theta - self.robot_theta
        if turn_angle > 180:
            turn_angle -= 360
        if turn_angle < -180:
            turn_angle += 360
        print("TURNING TO FACE HOME: {}".format(turn_angle))
        if abs(turn_angle) > 0.1:
            commands.send_drive_turn_command(turn_angle)

        from wait_state import WaitState
        return WaitState()

