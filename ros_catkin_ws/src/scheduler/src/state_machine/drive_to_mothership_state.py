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
import geometry_utils

class DriveToMothershipState(State):
    def __init__(self):
        super(DriveToMothershipState, self).__init__("Drive to Mothership State")

    def start(self):
        super(DriveToMothershipState, self).start()
        commands.set_display_state(commands.LETTER)

    def run(self):
        
        if globals.current_letter <= 2:
            drive_utils.go_to_point((globals.abc_approach_x, globals.abc_approach_y))
        else:   
            drive_utils.go_to_point((globals.def_approach_x, globals.def_approach_y))

        #drive_utils.wait_for_pose_update()
        turn_angle, _ = drive_utils.get_drive_instructions(\
                                (globals.mothership_x, globals.mothership_y)) 
        #print("TURNING TO FACE MOTHERSHIP: {}".format(turn_angle))
        drive_utils.turn(turn_angle)

        drive_utils.wait_for_pose_update()
        
        #TODO 4/1: Added to account for the mothership being very close to the edge of the board
        if geometry_utils.dist((drive_utils.robot_x, drive_utils.robot_y),\
            (globals.mothership_x, globals.mothership_y)) < 17:
            from place_in_slot_state import PlaceInSlotState
            return PlaceInSlotState()
        else:
            from approach_mothership_state import ApproachMothershipState
            return ApproachMothershipState(False)
