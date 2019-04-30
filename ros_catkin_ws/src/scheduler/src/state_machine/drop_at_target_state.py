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

class DropAtTargetState(State):
    def __init__(self):
        super(DropAtTargetState, self).__init__("Drop At Target State")

    def start(self):
        super(DropAtTargetState, self).start()
        commands.set_display_state(commands.LETTER)
        
        target_index = globals.current_letter
        self.target_point = (globals.target_x_coords[target_index], \
                             globals.target_y_coords[target_index])

        self.dropoff_dist = 4

    def run(self):
        
        #go to target approach point
        success = drive_utils.go_to_point(self.target_point)

        #face dropoff spot
        turn_angle = globals.target_facing_angle - drive_utils.robot_theta
        drive_utils.turn(turn_angle)

        #send drop block command
        commands.send_claw_command(commands.CARRY_ANGLE)
        t.sleep(0.5)
        commands.send_drop_block_command(self.dropoff_dist, 0)
        globals.block_queue.popleft()
        t.sleep(4)

        if (len(globals.block_queue) > 0):
            from drive_to_block_state import DriveToBlockState
            return DriveToBlockState()
        else:
            from wait_state import WaitState
            return WaitState()
