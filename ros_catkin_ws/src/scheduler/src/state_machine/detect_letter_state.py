import globals
from state import State
from std_msgs.msg import UInt8
from object_detection.srv import *
import commands
import rospy
import time as t

class DetectLetterState(State):
    def __init__(self):
        super(DetectLetterState, self).__init__("Detect Letter")

    def start(self):
        rospy.loginfo("Entering detect letter state")
        
    def run(self):
        t.sleep(5)
        try:
            detected_letter = commands.letter_srv()
            globals.current_letter = detected_letter.letter
            commands.display_letter(detected_letter.letter)
            commands.set_display_state(commands.LETTER)
            t.sleep(1)
        except Exception as e:
            print(e)
            return self

        # from put_down_block_state import * 
        # return PutDownBlockState()
        from find_mothership_state import *
        return FindMothershipState(False)

    def finish(self):
        rospy.loginfo("Exiting detect letter state")
