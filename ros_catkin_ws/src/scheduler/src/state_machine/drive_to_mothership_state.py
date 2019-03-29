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
        commands.set_display_state(commands.LETTER)

    def run(self):
        
        if globals.current_letter <= 2:
            drive_utils.go_to_point((globals.abc_x, globals.abc_y))
        else:   
            drive_utils.go_to_point((globals.def_x, globals.def_y))

        #drive_utils.wait_for_pose_update()
        turn_angle, _ = drive_utils.get_drive_instructions(\
                                (globals.mothership_x, globals.mothership_y)) 
        print("TURNING TO FACE MOTHERSHIP: {}".format(turn_angle))
        drive_utils.turn(turn_angle)

        from approach_mothership_state import ApproachMothershipState
        return ApproachMothershipState(False)
