import globals
from state import State
from std_msgs.msg import UInt8
from std_msgs.msg import Bool 
from geometry_msgs.msg import Pose2D
from object_detection.srv import *
import commands
import drive_utils
from geometry_utils import *
import time as t
import math
import rospy

class ReturnToHomeState(State):
    def __init__(self):
        super(ReturnToHomeState, self).__init__("Return To Home State")

    def start(self):
        super(ReturnToHomeState, self).start()
        commands.set_display_state(commands.NORMAL)
        self.home_x = 4.5*12 
        self.home_y = 4.5*12
        self.home_theta = 90

    def run(self):
        drive_utils.go_to_point((self.home_x, self.home_y))
        turn_angle = self.home_theta - drive_utils.robot_theta
        turn_angle = bound_angle(turn_angle)
        print("TURNING TO FACE HOME: {}".format(turn_angle))
        drive_utils.turn(turn_angle)

        from wait_state import WaitState
        return WaitState()

