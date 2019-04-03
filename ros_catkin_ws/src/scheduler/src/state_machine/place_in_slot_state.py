import globals
from state import State
import commands
import time as t
import rospy
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import Pose2D
import drive_utils
import geometry_utils
import math

class PlaceInSlotState(State):
    def __init__(self):
        super(PlaceInSlotState, self).__init__("Place in Slot")

    def start(self):
        super(PlaceInSlotState, self).start()

        #new_x, new_y = (globals.abc_approach_x, globals.abc_approach_y) if globals.current_letter <= 2 else\
        #               (globals.def_approach_x, globals.def_approach_y)
        #commands.send_override_position_command(new_x, new_y)

        self.forward_dist = 5 #inches
        slot_turn = 15

        drive_utils.wait_for_pose_update()
        robot_pos = (drive_utils.robot_x, drive_utils.robot_y)
        mothership_pos = (globals.mothership_x, globals.mothership_y)

        print("Dropping blocks: distance to mothership: {}", geometry_utils.dist(robot_pos, mothership_pos))
        if globals.current_letter not in [1,4]:
            self.forward_dist = self.forward_dist / math.cos(math.radians(slot_turn)) 
        self.turn_angle = 0
        if globals.current_letter % 3 == 0:
            self.turn_angle = slot_turn
        elif globals.current_letter % 3 == 2:
            self.turn_angle = -slot_turn

    def run(self):

        #drop it off
        commands.send_drop_block_command(self.forward_dist, self.turn_angle)
        drive_utils.wait_for_pose_change()

        globals.current_block += 1

        if globals.current_block == globals.num_blocks:
            from return_to_home_state import ReturnToHomeState
            return ReturnToHomeState()
        else:
            from drive_to_block_state import DriveToBlockState 
            return DriveToBlockState()
