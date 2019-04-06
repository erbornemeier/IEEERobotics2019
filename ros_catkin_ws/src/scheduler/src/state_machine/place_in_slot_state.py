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
    def __init__(self, is_dynamic=False):
        super(PlaceInSlotState, self).__init__("Place in Slot")
        self.is_dynamic = is_dynamic

    def start(self):
        super(PlaceInSlotState, self).start()

        #new_x, new_y = (globals.abc_approach_x, globals.abc_approach_y) if globals.current_letter <= 2 else\
        #               (globals.def_approach_x, globals.def_approach_y)
        #commands.send_override_position_command(new_x, new_y)

        drive_utils.wait_for_pose_update()
        robot_pos = (drive_utils.robot_x, drive_utils.robot_y)
        mothership_pos = (globals.mothership_x, globals.mothership_y)
        dist_to_mothership = geometry_utils.dist(robot_pos, mothership_pos)
        print("Dropping blocks: distance to mothership: {}".format(\
                                geometry_utils.dist(robot_pos, mothership_pos)))

        #move forward 5 if distance is 16.5ish, move forward 0 if distance is 16.5-5 = 11.5,
        #move forward 5 if distance is 15.0ish, move forward 0 if distance is 15.0-5 = 10.0,
        self.forward_dist = 5
        slot_turn = 15
        if self.is_dynamic:
            max_dist = 16.5
            min_dist = 8
            if dist_to_mothership > max_dist:
                dist_to_mothership = max_dist
            if dist_to_mothership < min_dist:
                dist_to_mothership = min_dist
            perc = (dist_to_mothership - min_dist) / (max_dist - min_dist)

            self.forward_dist = self.forward_dist * perc
            
        if globals.current_letter not in [1,4]:
            self.forward_dist = self.forward_dist / math.cos(math.radians(slot_turn)) 
        self.turn_angle = 0
        if globals.current_letter % 3 == 0:
            self.turn_angle = slot_turn
        elif globals.current_letter % 3 == 2:
            self.turn_angle = -slot_turn

        print("Place in Slot, forward dist: {}".format(self.forward_dist))

    def run(self):

        #drop it off
        commands.send_drop_block_command(self.forward_dist, self.turn_angle)
        drive_utils.wait_for_pose_change()

        globals.current_block += 1
        # globals.placed_blocks.append(globals.block_queue[0])
        globals.placed_blocks.append(globals.current_letter)
        globals.block_queue.popleft()

        if globals.current_block == globals.num_blocks:
            from return_to_home_state import ReturnToHomeState
            return ReturnToHomeState()
        else:
            from drive_to_block_state import DriveToBlockState 
            return DriveToBlockState()
