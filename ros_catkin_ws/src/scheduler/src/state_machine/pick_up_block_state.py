import globals
from state import State
import commands
import time as t
import rospy
import drive_utils
from std_msgs.msg import UInt8, Bool

class PickUpBlockState(State):
    def __init__(self):
        super(PickUpBlockState, self).__init__("Pick Up Block")

    def start(self):
        super(PickUpBlockState, self).start()

    def run(self):
        commands.send_pickup_command()
        
        current_block = globals.block_queue[0]
        drive_utils.remove_bad_points_around_block(current_block[0], current_block[1])

        from detect_letter_state import DetectLetterState 
        return DetectLetterState()
