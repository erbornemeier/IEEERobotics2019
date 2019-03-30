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
        

        drive_utils.remove_bad_points_around_block(\
                    globals.x_coords[globals.current_block],\
                    globals.y_coords[globals.current_block])
        from detect_letter_state import DetectLetterState 
        return DetectLetterState()
