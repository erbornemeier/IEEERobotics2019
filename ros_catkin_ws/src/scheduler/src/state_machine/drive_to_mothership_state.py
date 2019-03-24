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
import drive_utils

class DriveToMothershipState(State):
    def __init__(self):
        super(DriveToMothershipState, self).__init__("Drive to Mothership State")

    def start(self):
        super(DriveToMothershipState, self).start()

        self.pose_sub = rospy.Subscriber('robot_pose', Pose2D, self.__set_pose__)

        self.robot_x = -1
        self.robot_y = -1
        self.robot_theta = -1

        self.close_dist = 18 #inches

        t.sleep(0.5)
        commands.set_display_state(commands.NORMAL)

    def __set_pose__(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta

    def __get_close_to_block__(self):
        # wait until robot pos recieved
        t.sleep(1)
        
        if globals.current_letter <= 2:
            int_point = drive_utils.drive_safely((globals.abc_x, globals.abc_y))
        else:   
            int_point = drive_utils.drive_safely((globals.def_x, globals.def_y))
        print("INT POINT: {}".format(int_point))
        if int_point is not None:
            drive_utils.go_to_point(int_point)

        if globals.current_letter <= 2:
            drive_utils.go_to_point((globals.abc_x, globals.abc_y))
        else:   
            drive_utils.go_to_point((globals.def_x, globals.def_y))
       
        t.sleep(1)
        self.robot_x = -1
        while (self.robot_x == -1):
            print("waiting for pose")
            t.sleep(0.5)

        
        dx, dy = globals.mothership_x - self.robot_x, globals.mothership_y-self.robot_y
        turn_angle = (math.atan2(dy, dx) * 180.0/3.14159) - self.robot_theta
        if turn_angle > 180:
            turn_angle -= 360
        if turn_angle < -180:
            turn_angle += 360
        print("TURNING TO FACE MOTHERSHIP: {}".format(turn_angle))
        if abs(turn_angle) > 0.1:
            commands.send_drive_turn_command(turn_angle)
            t.sleep(5)

    def run(self):
        t.sleep(2)

        self.__get_close_to_block__()

        from approach_mothership_state import ApproachMothershipState
        return ApproachMothershipState(False)
