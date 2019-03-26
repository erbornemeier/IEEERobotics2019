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
        #close claw and lift it up
        #t.sleep(0.5) 
        #commands.send_grip_command(commands.CLAW_CLOSED)
        #t.sleep(0.5)
        #commands.send_claw_command(commands.PICKUP_ANGLE)
        #t.sleep(0.5)

        #drop it off
        self.__drop_off_at_letter__()
        commands.send_drop_block_command(self.forward_dist, self.turn_angle)
        globals.current_block += 1

        if globals.current_block == globals.num_blocks:
            from return_to_home_state import ReturnToHomeState
            return ReturnToHomeState()
        else:
            from drive_to_block_state import DriveToBlockState 
            return DriveToBlockState()
