import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D

class PlaceInSlotState(State):
    def __init__(self):
        super(PlaceInSlotState, self).__init__("Place in Slot")

    def start(self):
        super(PlaceInSlotState, self).start()

        #new_x, new_y = (globals.abc_approach_x, globals.abc_approach_y) if globals.current_letter <= 2 else\
        #               (globals.def_approach_x, globals.def_approach_y)
        #commands.send_override_position_command(new_x, new_y)

        self.forward_dist = 5 #inches
        extra_dist = 0.5
        if globals.current_letter not in [1,4]:
            self.forward_dist += extra_dist
        self.turn_angle = 0
        if globals.current_letter % 3 == 0:
            self.turn_angle = 15 
        elif globals.current_letter % 3 == 2:
            self.turn_angle = -15

    def run(self):

        #drop it off
        t.sleep(0.5)
        commands.send_drop_block_command(self.forward_dist, self.turn_angle)
        globals.current_block += 1
        #TODO when done?
        t.sleep(5)

        if globals.current_block == globals.num_blocks:
            from return_to_home_state import ReturnToHomeState
            return ReturnToHomeState()
        else:
            from drive_to_block_state import DriveToBlockState 
            return DriveToBlockState()
